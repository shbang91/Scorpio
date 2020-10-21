#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include <Eigen/Dense>

#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <Utils/IO/IOUtilities.hpp>

namespace gazebo
{
    class ScorpioPlugin : public ModelPlugin
    {
        public:
            ScorpioPlugin(){
                total_num_joint_idx_ = Eigen::VectorXd::Zero(11);
                active_joint_idx_ = Eigen::VectorXd::Zero(7);
                q_ = Eigen::VectorXd::Zero(11);
                qdot_ = Eigen::VectorXd::Zero(11);

                interface_ = new ScorpioInterface();
                sensordata_ = new ScorpioSensorData();
                command_ = new ScorpioCommand();
            }

            ~ScorpioPlugin(){
                delete interface_;
                delete sensordata_;
                delete command_;
            }

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
            {
                this->model = _parent; //Store the pointer to the model

                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                        std::bind(&ScorpioPlugin::OnUpdate, this)); // Listen to the update event
                                                                    // This event is broadcast every simulation iteration
                std::cout << "==================================================" << std::endl;
                std::cout << "The scorpio_plugin is attached to model[" << this->model->GetName()
                    << "]"<< std::endl;

                physics::Joint_V joints = this->model->GetJoints();
                physics::Link_V links = this->model->GetLinks();

                //std::cout << "====================" << std::endl;
                //std::cout << "joint name" << std::endl;
                //for (int i = 0; i < joints.size(); ++i) {
                   //std::cout << joints[i]->GetName() << std::endl;
                //}

                //std::cout << "====================" << std::endl;
                //std::cout << "link name" << std::endl;
                //for (int i = 0; i < links.size(); ++i) {
                   //std::cout << links[i]->GetName() << std::endl;
                //}
                //std::cout << "====================" << std::endl;

                //Closed loop
                total_num_joint_idx_ << 1,2,3,4,5,6,7,8,9,10,11;
                active_joint_idx_ << 1,2,5,6,9,10,11;

                if (!ros::isInitialized())
                {
                    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                    return;
                }

                this->rosNode.reset(new ros::NodeHandle("scorpio"));
                this->rosSub = this->rosNode->subscribe("moveAbsolute", 1000, &ScorpioPlugin::OnRosMsg, this);

            }
            // Called by the world update start event
            void OnUpdate() {

                //double seconds_per_step = this ->model->Getworld()->GetPhysicsEngine()->GetUpdatePeriod();
                //double alternate_time = this->model->Getworld()->GetSimTime().Double();
                //iters = this->model->GetWorld()->GetIterations();

                gazebo::physics::Joint_V joints = this->model->GetJoints();

                for (int i = 0; i < total_num_joint_idx_.size(); ++i) {
                    q_[i] = joints[total_num_joint_idx_[i]]->Position(0);
                    qdot_[i] = joints[total_num_joint_idx_[i]] ->GetVelocity(0);
                }
                //myUtils::pretty_print(q_, std::cout, "joint pos");
                //myUtils::pretty_print(qdot_, std::cout, "joint vel");

                sensordata_->q = q_;
                sensordata_->qdot = qdot_;
                interface_ -> getCommand(sensordata_,command_);

                for (int i = 0; i < active_joint_idx_.size(); ++i) {
                   joints[active_joint_idx_[i]]->SetForce(0,command_->jtrq[i]);
                   //joints[active_joint_idx_[i]]->SetForce(0,-qdot_[i]);
                   //joints[active_joint_idx_[i]]->SetForce(0,0);
                }
            }

            void OnRosMsg(geometry_msgs::Pose pose) {
                ROS_INFO("Moving To Point!");

                geometry_msgs::Point p  = pose.position;
                geometry_msgs::Quaternion q  = pose.orientation;
                std::cout << p.x << std::endl;
                std::cout << p.y << std::endl;
                std::cout << p.z << std::endl;

                ((ScorpioInterface*)interface_)->MoveEndEffectorTo(p.x, p.y, p.z, q.x, q.y, q.z, q.w);
            }


        private:
            physics::ModelPtr model; //pointer to the model
            event::ConnectionPtr updateConnection; //Pointer to the update event connection

            Eigen::VectorXd active_joint_idx_;
            Eigen::VectorXd total_num_joint_idx_;
            Eigen::VectorXd q_;
            Eigen::VectorXd qdot_;

            ScorpioInterface* interface_;
            ScorpioSensorData* sensordata_;
            ScorpioCommand* command_;
            //Ros Stuff
            std::unique_ptr<ros::NodeHandle> rosNode;
            ros::Subscriber rosSub;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ScorpioPlugin)
}
