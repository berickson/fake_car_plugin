#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int32.h"


#include "std_msgs/Float64.h"
#include "sensor_msgs/Joy.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include "math.h"

class BicycleModel {
    double wheelbase_length;
    double steer_angle;
public:
    BicycleModel(double wheelbase_length, double steer_angle = 0.0) {
        this->wheelbase_length = wheelbase_length;
        this->steer_angle = steer_angle;
    }
    
    void set_steer_angle(double steer_angle) {
        this->steer_angle = steer_angle;
    }
    
    void set_rear_curvature(double k_rear) {
        this->steer_angle = atan(k_rear * this->wheelbase_length);
    }
    
    double get_rear_curvature(){
        return tan(this->steer_angle) / this->wheelbase_length;
    }
    
    void set_front_curvature(double k_front){
        this->steer_angle = asin(this->wheelbase_length * k_front);
    }

    double get_front_curvature() {
        return sin(this->steer_angle) / this->wheelbase_length;
    }
    
    double get_steer_angle() {
        return this->steer_angle;
    }
    
    BicycleModel get_offset_bicycle(double delta_y){
        if(this->steer_angle == 0.0) {
            return BicycleModel(wheelbase_length, 0.0);
        }
        BicycleModel new_bike(this->wheelbase_length);
        new_bike.set_rear_curvature(1./(1./this->get_rear_curvature() - delta_y));
        return new_bike;
    }
};

class AckermannModel : public BicycleModel {
    double front_wheelbase_width;
public:
    AckermannModel(double wheelbase_length, double front_wheelbase_width, double  steer_angle = 0.0)
        : BicycleModel(wheelbase_length, steer_angle)
    {
        this->front_wheelbase_width = front_wheelbase_width;
    }

    BicycleModel get_left_bicycle() {
        return this->get_offset_bicycle(this->front_wheelbase_width / 2.);
    }

    BicycleModel get_right_bicycle() {
        return this->get_offset_bicycle(-1. * this->front_wheelbase_width / 2.);
    }
};


namespace gazebo  {
    class FakeCarPlugin : public ModelPlugin 
    {
        double steer = 0;
        double velocity = 0;
        const double wheelbase_length = 0.3429 ;
        const double front_wheelbase_width = 0.25;
        const double rear_wheelbase_width = 0.25;
        AckermannModel car_model={0.25, 0.25};



        physics::ModelPtr model;
        physics::JointControllerPtr jc;
        physics::JointPtr fl_str_joint;
        physics::JointPtr fr_str_joint;
        physics::JointPtr fl_axle_joint;
        physics::JointPtr fr_axle_joint;
        physics::JointPtr bl_axle_joint;
        physics::JointPtr br_axle_joint;

        common::PID fl_pid, fr_pid, bl_pid, br_pid;
        std::unique_ptr<ros::NodeHandle> n;
        ros::Subscriber fl_sub, fr_sub, ackermann_sub, twist_sub, joy_sub;
        ros::Publisher odo_fl_pub, odo_fr_pub, ackermann_pub;

        ros::CallbackQueue ros_queue;

        std::thread ros_queue_thread;

        void publish_state(){
            const int ticks_per_revolution = 42;

            std_msgs::Int32 odo_fl;
            odo_fl.data = (int) fl_axle_joint->Position() * ticks_per_revolution / (M_2_PI);
            odo_fl_pub.publish(odo_fl);

            std_msgs::Int32 odo_fr;
            odo_fr.data = (int) fr_axle_joint->Position() * ticks_per_revolution / (M_2_PI);
            odo_fr_pub.publish(odo_fr);
        }

        void joy_callback(sensor_msgs::Joy msg) {
            ackermann_msgs::AckermannDriveStamped ad;
            ad.drive.steering_angle = msg.axes[3];
            ad.drive.speed = msg.axes[1] * 100;
            ackermann_pub.publish(ad);
        }

        void ackermann_callback(ackermann_msgs::AckermannDriveStamped msg) {
            car_model.set_steer_angle(msg.drive.steering_angle);

            jc->SetPositionTarget(
                fl_str_joint->GetScopedName(), car_model.get_left_bicycle().get_steer_angle());

            jc->SetPositionTarget(
                fr_str_joint->GetScopedName(), car_model.get_right_bicycle().get_steer_angle());

            double curvature = car_model.get_rear_curvature();
            double speed = msg.drive.speed;
            if(curvature == 0){
                jc->SetVelocityTarget(
                    bl_axle_joint->GetScopedName(), speed);
                jc->SetVelocityTarget(
                    br_axle_joint->GetScopedName(), speed);
            } else {
                double radius = 1./curvature;
                double left_radius = radius - rear_wheelbase_width / 2.;
                double right_radius = radius + rear_wheelbase_width / 2.;

                jc->SetVelocityTarget(
                    bl_axle_joint->GetScopedName(), speed*left_radius/radius);
                jc->SetVelocityTarget(
                    br_axle_joint->GetScopedName(), speed*right_radius/radius);
            }
        }

        void twist_callback(geometry_msgs::Twist msg) {
            ackermann_msgs::AckermannDriveStamped ad;
            ad.drive.speed = msg.linear.x;
        
            if (msg.linear.x == 0) {
                ad.drive.steering_angle = 0;
            }
            else {
                car_model.set_rear_curvature(msg.angular.z / msg.linear.x);
                ad.drive.steering_angle = car_model.get_steer_angle();
            }
            ackermann_pub.publish(ad);
        }

        private: void QueueThread()
        {
            ros::Rate loop_rate(100);

            while (n->ok())
            {
                ros_queue.callAvailable();
                publish_state();
                loop_rate.sleep();
            }
        }
        
        public: FakeCarPlugin() : ModelPlugin()
        {
            ROS_INFO("FakeCarPlugin() : ModelPlugin()");
        }


        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            model = _model;
            ROS_INFO("Connected to model %s", model->GetName().c_str());
            
            jc = model->GetJointController();

            // front left str
            fl_pid = common::PID(1, 0, 0);
            fl_str_joint  = model->GetJoint("front_left_wheel_steer_joint");

            jc->SetPositionPID(
                fl_str_joint->GetScopedName(), fl_pid);

            // front right str            
            fr_pid = common::PID(1, 0, 0);
            fr_str_joint  = _model->GetJoint("front_right_wheel_steer_joint");
            jc->SetPositionPID(
                fr_str_joint->GetScopedName(), fr_pid);

            fl_axle_joint = model->GetJoint("front_left_wheel_joint");
            fr_axle_joint = model->GetJoint("front_right_wheel_joint");

            // back left speed
            bl_pid = common::PID(0.1, 0.01, 0.0);
            bl_axle_joint = model->GetJoint("back_left_wheel_joint");
            jc->SetVelocityPID(
                bl_axle_joint->GetScopedName(), bl_pid);
            

            br_pid = common::PID(0.1, 0.01, 0.0);
            br_axle_joint = model->GetJoint("back_right_wheel_joint");
            jc->SetVelocityPID(
                br_axle_joint->GetScopedName(), br_pid);
            


            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "fake_car_plugin",
                    ros::init_options::NoSigintHandler);
            }

            
            n.reset(new ros::NodeHandle("fake_car_plugin"));

            // publish
            odo_fl_pub = n->advertise<std_msgs::Int32>("/" + model->GetName() + "/odo_fl", 10);
            odo_fr_pub = n->advertise<std_msgs::Int32>("/" + model->GetName() + "/odo_fr", 10);
            ackermann_pub = n->advertise<ackermann_msgs::AckermannDriveStamped>("/" + model->GetName() + "/cmd_ackermann", 10);

            // subscribe
            joy_sub = n->subscribe<sensor_msgs::Joy>(
                "/joy",
                2,
                &FakeCarPlugin::joy_callback, this);

            ackermann_sub = n->subscribe<ackermann_msgs::AckermannDriveStamped>(
                "/" + model->GetName() + "/cmd_ackermann",
                2,
                &FakeCarPlugin::ackermann_callback, this);



            ros_queue_thread = std::thread(std::bind(&FakeCarPlugin::QueueThread, this));
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(FakeCarPlugin)
}

