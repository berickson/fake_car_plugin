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



        private: physics::ModelPtr model;
        private: physics::JointPtr fl_str_joint;
        private: physics::JointPtr fr_str_joint;
        private: physics::JointPtr fl_axle_joint;
        private: physics::JointPtr fr_axle_joint;
        private: common::PID fl_pid, fr_pid;
        private: std::unique_ptr<ros::NodeHandle> ros_node;
        private: ros::Subscriber rosSub, fl_sub, fr_sub, ackermann_sub, twist_sub, joy_sub;
        private: ros::Publisher odo_fl_pub, odo_fr_pub;

        private: ros::CallbackQueue ros_queue;

        private: std::thread ros_queue_thread;

        private: void publish_state(){
            const int ticks_per_revolution = 42;

            std_msgs::Int32 odo_fl;
            odo_fl.data = (int) fl_axle_joint->Position() * ticks_per_revolution / (M_2_PI);
            odo_fl_pub.publish(odo_fl);

            std_msgs::Int32 odo_fr;
            odo_fr.data = (int) fr_axle_joint->Position() * ticks_per_revolution / (M_2_PI);
            odo_fr_pub.publish(odo_fr);
        }


        ros::Publisher ackermann_pub;
        ros::Publisher front_right_steer_pub;
        ros::Publisher front_left_steer_pub;
        ros::Publisher back_left_speed_pub;
        ros::Publisher back_right_speed_pub;


        void joy_callback(sensor_msgs::Joy msg) {
            ackermann_msgs::AckermannDriveStamped ad;
            ad.drive.steering_angle = msg.axes[3];
            ad.drive.speed = msg.axes[1] * 100;
            ackermann_pub.publish(ad);
            ROS_INFO("joy_callback");
        }

        void ackermann_callback(ackermann_msgs::AckermannDriveStamped msg) {
            car_model.set_steer_angle(msg.drive.steering_angle);

            std_msgs::Float64 right_angle;
            right_angle.data = car_model.get_right_bicycle().get_steer_angle();
            front_right_steer_pub.publish(right_angle);

            std_msgs::Float64 left_angle;
            left_angle.data = car_model.get_left_bicycle().get_steer_angle();
            front_left_steer_pub.publish(left_angle);

            
            double curvature = car_model.get_rear_curvature();
            if(curvature == 0){
                std_msgs::Float64 center_speed;
                center_speed.data = msg.drive.speed;
                back_left_speed_pub.publish(center_speed);
                back_right_speed_pub.publish(center_speed);
            } else {
                double radius = 1./curvature;
                double left_radius = radius - rear_wheelbase_width / 2.;
                double right_radius = radius + rear_wheelbase_width / 2.;

                std_msgs::Float64 left_speed;
                std_msgs::Float64 right_speed;

                left_speed.data = msg.drive.speed*left_radius/radius;
                right_speed.data = msg.drive.speed*right_radius/radius;

                back_left_speed_pub.publish(left_speed);
                back_right_speed_pub.publish(right_speed);
            }
            ROS_INFO("ackermann_callback");
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

        public: void on_fl_str(const std_msgs::Float64ConstPtr &_msg)
        {
            model->GetJointController()->SetPositionTarget(
                fl_str_joint->GetScopedName(), _msg->data);
        }

        public: void on_fr_str(const std_msgs::Float64ConstPtr &_msg)
        {
            model->GetJointController()->SetPositionTarget(
                fr_str_joint->GetScopedName(), _msg->data);
        }

        private: void QueueThread()
        {
            static const double timeout = 0.001;
            ros::Rate loop_rate(100);

            while (ros_node->ok())
            {
                ros_queue.callAvailable(ros::WallDuration(timeout));
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
            ROS_INFO("hello fake_car!");

            ROS_INFO("Connected to model %s", _model->GetName().c_str());
            model = _model;

            // front left str
            fl_pid = common::PID(1, 0, 0);
            ROS_INFO("created pid");
            fl_str_joint  = _model->GetJoint("front_left_wheel_steer_joint");
            ROS_INFO("Found fl_str_joint %s", fl_str_joint->GetScopedName().c_str());

            model->GetJointController()->SetPositionPID(
                fl_str_joint->GetScopedName(), fl_pid);

            // front right str            
            fr_pid = common::PID(1, 0, 0);
            ROS_INFO("created pid");
            fr_str_joint  = _model->GetJoint("front_right_wheel_steer_joint");
            ROS_INFO("Found fr_str_joint %s", fr_str_joint->GetScopedName().c_str());

            fl_axle_joint = _model->GetJoint("front_left_wheel_joint");
            fr_axle_joint = _model->GetJoint("front_right_wheel_joint");

            model->GetJointController()->SetPositionPID(
                fr_str_joint->GetScopedName(), fr_pid);

            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "fake_car_plugin",
                    ros::init_options::NoSigintHandler);
            }

            
            ros_node.reset(new ros::NodeHandle("fake_car_plugin"));

            // publish
            odo_fl_pub = ros_node->advertise<std_msgs::Int32>("/" + model->GetName() + "/odo_fl", 10);
            odo_fr_pub = ros_node->advertise<std_msgs::Int32>("/" + model->GetName() + "/odo_fr", 10);
            ackermann_pub = ros_node->advertise<ackermann_msgs::AckermannDriveStamped>("/cmd_ackermann", 10);
            front_right_steer_pub = ros_node->advertise<std_msgs::Float64>("/fake_car/fr_str", 10);
            front_left_steer_pub = ros_node->advertise<std_msgs::Float64>("/fake_car/fl_str", 10);

            back_left_speed_pub = ros_node->advertise<std_msgs::Float64>("/fake_car/back_left_wheel_velocity_controller/command", 10);
            back_right_speed_pub = ros_node->advertise<std_msgs::Float64>("/fake_car/back_right_wheel_velocity_controller/command", 10);

            // subscribe

            fl_sub = ros_node->subscribe<std_msgs::Float64>(
                "/" + model->GetName() + "/fl_str",
                2,
                &FakeCarPlugin::on_fl_str, this);

            fr_sub = ros_node->subscribe<std_msgs::Float64>(
                "/" + model->GetName() + "/fr_str",
                2,
                &FakeCarPlugin::on_fr_str, this);

            joy_sub = ros_node->subscribe<sensor_msgs::Joy>(
                "/joy",
                2,
                &FakeCarPlugin::joy_callback, this);

            ackermann_sub = ros_node->subscribe<ackermann_msgs::AckermannDriveStamped>(
                "/cmd_ackermann",
                2,
                &FakeCarPlugin::ackermann_callback, this);



            ros_queue_thread = std::thread(std::bind(&FakeCarPlugin::QueueThread, this));
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(FakeCarPlugin)
}

