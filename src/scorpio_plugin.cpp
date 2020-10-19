#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>

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
                
                if(!ros::isInitialized()){
                ROS_FATAL_STREAM("A ROS node for GAZEBO has not been initialized, unable to load plugin");
                }

                this->rosNode_.reset(new ros::NodeHandle("scorpio"));
                //this->rosSub_ = this->rosNode->subscribe("moveAbsolute", 1000, &ScorpioPlugin::OnRosMsg, this);

            }
            // Called by the world update start event
            void OnUpdate() {

                gazebo::physics::Joint_V joints = this->model->GetJoints();

                for (int i = 0; i < total_num_joint_idx_.size(); ++i) {
                    q_[i] = joints[total_num_joint_idx_[i]]->Position(0);
                    qdot_[i] = joints[total_num_joint_idx_[i]] ->GetVelocity(0);
                }
                //myUtils::pretty_print(q_, std::cout, "joint pos");
                //myUtils::pretty_print(qdot_, std::cout, "joint vel");

                // ================================================
                // Scorpio grasp APIs Examples
                // ================================================

                static bool b_move_cmd(true);
                if (((ScorpioInterface*)interface_)->IsReadyToMove() && b_move_cmd) {
                    std::cout << "First Moving Command Received" << std::endl;
                    ((ScorpioInterface*)interface_)->MoveEndEffectorTo(-0.5, 0.1, 1.3, 0.7071, 0.000316,-0.7071,0.00025); //Maintain initial orientation
                    b_move_cmd = false;
                }

                static bool b_grasp_cmd(true);
                if (((ScorpioInterface*)interface_)->IsReadyToGrasp() && b_grasp_cmd) {
                    std::cout << "First Grasping Command Received" << std::endl;
                    ((ScorpioInterface*)interface_)->Grasp();
                    b_grasp_cmd = false;
                }

                static bool b_move_while_hold_cmd(true);
                if (((ScorpioInterface*)interface_)->IsReadyToMove() && b_move_while_hold_cmd) {
                    std::cout << "First Moving While Holding Command Received" << std::endl;
                    ((ScorpioInterface*)interface_)->MoveEndEffectorTo(-0.3, -0.5, 1.5, 0.7071,0.000316,-0.7071,0.00025);
                    b_move_while_hold_cmd = false;
                }

                static bool b_release_cmd(true);
                if (((ScorpioInterface*)interface_)->IsReadyToMove() && b_release_cmd) {
                    std::cout << "First Pouring Command Received" << std::endl;
                    ((ScorpioInterface*)interface_)->MoveEndEffectorTo(-0.3, -0.5, 1.5, 0.7073, -0.7070, 0, 0);
                    b_release_cmd = false;
                }

                sensordata_->q = q_;
                sensordata_->qdot = qdot_;
                interface_ -> getCommand(sensordata_,command_); 

                for (int i = 0; i < active_joint_idx_.size(); ++i) {
                   joints[active_joint_idx_[i]]->SetForce(0,command_->jtrq[i]); 
                   //std::cout << command_->jtrq[i] << std::endl;
                   //joints[active_joint_idx_[i]]->SetForce(0,-qdot_[i]); 
                   //joints[active_joint_idx_[i]]->SetForce(0,0); 
                }
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

            //Ros stuff
            std::unique_ptr<ros::NodeHandle> rosNode_;
            ros::Subscriber rosSub_;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ScorpioPlugin)
}
