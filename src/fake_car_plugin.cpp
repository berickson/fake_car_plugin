#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo  {
    class FakeCarPlugin : public ModelPlugin 
    {
        private: physics::ModelPtr model;
        private: physics::JointPtr fl_str_joint;
        private: physics::JointPtr fr_str_joint;
        private: common::PID fl_pid, fr_pid;
        private: std::unique_ptr<ros::NodeHandle> ros_node;
        private: ros::Subscriber rosSub, fl_sub, fr_sub;

        /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue ros_queue;

        /// \brief A thread the keeps running the ros_queue
        private: std::thread ros_queueThread;


        public: void on_fl_str(const std_msgs::Float32ConstPtr &_msg)
        {
            model->GetJointController()->SetPositionTarget(
                fl_str_joint->GetScopedName(), _msg->data);
        }

        public: void on_fr_str(const std_msgs::Float32ConstPtr &_msg)
        {
            model->GetJointController()->SetPositionTarget(
                fr_str_joint->GetScopedName(), _msg->data);
        }

        /// \brief ROS helper function that processes messages
        private: void QueueThread()
        {
        static const double timeout = 0.01;
        while (ros_node->ok())
        {
            ros_queue.callAvailable(ros::WallDuration(timeout));
        }
        }
        public:
        FakeCarPlugin() : ModelPlugin()
        {
            ROS_INFO("FakeCarPlugin() : ModelPlugin()");
        }


        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Make sure the ROS node for Gazebo has already been initialized                                                                                    
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

            model->GetJointController()->SetPositionPID(
                fr_str_joint->GetScopedName(), fr_pid);

            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized())
            {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "fake_car_plugin",
                ros::init_options::NoSigintHandler);
            }

            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            
            ros_node.reset(new ros::NodeHandle("fake_car_plugin"));

            fl_sub = ros_node->subscribe<std_msgs::Float32>(
                "/" + model->GetName() + "/fl_str",
                1,
                &FakeCarPlugin::on_fl_str, this);

            fr_sub = ros_node->subscribe<std_msgs::Float32>(
                "/" + model->GetName() + "/fr_str",
                1,
                &FakeCarPlugin::on_fr_str, this);
/*
            rosSub = ros_node->subscribe(ros::SubscribeOptions::create<std_msgs::Float32>(
                "/" + model->GetName() + "/fl_str",
                1,
                boost::bind(&FakeCarPlugin::on_fl_str, this, _1),
                ros::VoidPtr(), &ros_queue));

            rosSub = ros_node->subscribe(ros::SubscribeOptions::create<std_msgs::Float32>(
                "/" + model->GetName() + "/fr_str",
                1,
                boost::bind(&FakeCarPlugin::on_fr_str, this, _1),
                ros::VoidPtr(), &ros_queue));
*/
            ros_queueThread = std::thread(std::bind(&FakeCarPlugin::QueueThread, this));
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(FakeCarPlugin)
}

