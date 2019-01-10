#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

namespace gazebo  {
    class FakeCarPlugin : public ModelPlugin 
    {
        /// \brief Pointer to the model.
        private: physics::ModelPtr model;

        /// \brief Pointer to the joint.
        private: physics::JointPtr joint;

        /// \brief A PID controller for the joint.
        private: common::PID pid;


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
            this->model = _model;
            pid = common::PID(0.1, 0, 0);
            ROS_INFO("created pid");
            joint  = _model->GetJoint("front_left_wheel_steer_joint");
            ROS_INFO("Found joint %s", this->joint->GetScopedName().c_str());

            this->model->GetJointController()->SetPositionPID(
                this->joint->GetScopedName(), this->pid);
            this->model->GetJointController()->SetPositionTarget(
                this->joint->GetScopedName(), 1.0);
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(FakeCarPlugin)
}

