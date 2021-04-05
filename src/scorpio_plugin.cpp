#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/GripperCommandGoal.h"

#include <Eigen/Dense>

#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <Utils/IO/IOUtilities.hpp>

#include <gazebo_scorpio_plugin/MoveEndEffectorToSrv.h>
#include <gazebo_scorpio_plugin/GripperCommandSrv.h>
#include <gazebo_scorpio_plugin/InterruptSrv.h>
#include <actionlib/client/simple_action_client.h>

namespace gazebo {
    class ScorpioPlugin : public ModelPlugin {
    public:
        ScorpioPlugin() {
            total_num_joint_idx_ = Eigen::VectorXd::Zero(11);
            active_joint_idx_ = Eigen::VectorXd::Zero(7);
            q_ = Eigen::VectorXd::Zero(11);
            qdot_ = Eigen::VectorXd::Zero(11);

            interface_ = new ScorpioInterface();
            sensordata_ = new ScorpioSensorData();
            command_ = new ScorpioCommand();
        }

        ~ScorpioPlugin() {
            delete interface_;
            delete sensordata_;
            delete command_;
        }

        void MoveEndEffectorTo(geometry_msgs::Pose pose) {
            if (((ScorpioInterface *) interface_)->IsReadyToMove()) {
                std::cout << "First Moving Command Received" << std::endl;
                ((ScorpioInterface *) interface_)->MoveEndEffectorTo(pose.position.x, pose.position.y, pose.position.z,
                                                                     pose.orientation.x, pose.orientation.y,
                                                                     pose.orientation.z, pose.orientation.w);
            }
        }

        bool InterruptSrv(gazebo_scorpio_plugin::InterruptSrv::Request &req,
                          gazebo_scorpio_plugin::InterruptSrv::Response &res){
            if (!((ScorpioInterface*) interface_)->IsReadyToMove()) {
               std::cout << "Interruption command Received" << std::endl;
               ((ScorpioInterface*) interface_)->Interrupt();
                res.success = true;
            }else{
            res.success = false;
            }
        }

        bool MoveEndEffectorToSrv(gazebo_scorpio_plugin::MoveEndEffectorToSrv::Request &req,
                                  gazebo_scorpio_plugin::MoveEndEffectorToSrv::Response &res) {
            if (((ScorpioInterface *) interface_)->IsReadyToMove()) {
                std::cout << "First Moving Command Received" << std::endl;
                // res.time = ((ScorpioInterface *) interface_)->GetTimeParam()
                ((ScorpioInterface *) interface_)->MoveEndEffectorTo(req.ee_pose.position.x, req.ee_pose.position.y, req.ee_pose.position.z,
                                                                     req.ee_pose.orientation.x, req.ee_pose.orientation.y,
                                                                     req.ee_pose.orientation.z, req.ee_pose.orientation.w);
                res.success = true;
                // TODO: either implement on ScorpioInterface/PnC side or find where param for action time is and modify here appropriately
                // sleep(10);
                // Alternate solution, wait for gripper to be ready to move again to return
                while (!((ScorpioInterface *) interface_)->IsReadyToMove())
                    sleep(1);
            } else {
                res.success = false;
            }
            res.ee_pose.position.x = (interface_->endeff_pos_)[0];
            res.ee_pose.position.y = (interface_->endeff_pos_)[1];
            res.ee_pose.position.z = (interface_->endeff_pos_)[2];
            res.ee_pose.orientation.x = (interface_->endeff_ori_).x();
            res.ee_pose.orientation.y = (interface_->endeff_ori_).y();
            res.ee_pose.orientation.z = (interface_->endeff_ori_).z();
            res.ee_pose.orientation.w = (interface_->endeff_ori_).w();
            return true;
        }

        bool GripperCommandSrv(gazebo_scorpio_plugin::GripperCommandSrv::Request &req,
                                  gazebo_scorpio_plugin::GripperCommandSrv::Response &res) {
            actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripperClient_("/scorpio_with_camera/robotiq_2f_85_gripper_controller/gripper_cmd", true);
            control_msgs::GripperCommandGoal goal;
            goal.command.position = req.command.position;
            // this->gripperClient_.sendGoal(goal);
            // bool finished_before_timeout = this->gripperClient_.waitForResult(ros::Duration(10.0));
            gripperClient_.sendGoal(goal);
            bool finished_before_timeout = gripperClient_.waitForResult(ros::Duration(10.0));

            res.success = finished_before_timeout;
            // todo: return actual resulting gripper width if desired
            return true;
        }

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
            this->model = _parent; //Store the pointer to the model

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&ScorpioPlugin::OnUpdate, this)); // Listen to the update event
            // This event is broadcast every simulation iteration
            std::cout << "==================================================" << std::endl;
            std::cout << "The scorpio_plugin is attached to model[" << this->model->GetName()
                      << "]" << std::endl;

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
            total_num_joint_idx_ << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
            active_joint_idx_ << 1, 2, 5, 6, 9, 10, 11;

            if (!ros::isInitialized()) {
                ROS_FATAL_STREAM("A ROS node for GAZEBO has not been initialized, unable to load plugin");
            }

            this->rosNode_.reset(new ros::NodeHandle("scorpio"));
            this->rosSub_ = this->rosNode_->subscribe("MoveEndEffectorTo", 1000, &ScorpioPlugin::MoveEndEffectorTo,
                                                      this);
            this->jointPub_ = this->rosNode_->advertise<sensor_msgs::JointState>("joint_pos", 10, this);
            this->endeffPub_ = this->rosNode_->advertise<geometry_msgs::Pose>("endeff_pos", 10, this);
            this->endeffServer_ = this->rosNode_->advertiseService("MoveEndEffectorToSrv", &ScorpioPlugin::MoveEndEffectorToSrv, this);
            this->gripperServer_ = this->rosNode_->advertiseService("GripperCommandSrv", &ScorpioPlugin::GripperCommandSrv, this);
            this->interruptServer_ = this->rosNode_->advertiseService("InterruptSrv", &ScorpioPlugin::InterruptSrv, this);
        }

        // Called by the world update start event
        void OnUpdate() {

            gazebo::physics::Joint_V joints = this->model->GetJoints();

            for (int i = 0; i < total_num_joint_idx_.size(); ++i) {
                q_[i] = joints[total_num_joint_idx_[i]]->Position(0);
                qdot_[i] = joints[total_num_joint_idx_[i]]->GetVelocity(0);
            }
            //myUtils::pretty_print(q_, std::cout, "joint pos");
            //myUtils::pretty_print(qdot_, std::cout, "joint vel");

            // ================================================
            // Scorpio grasp APIs Examples
            // ================================================
            //
            // static bool b_move_cmd(true);
            // if (((ScorpioInterface*)interface_)->IsReadyToMove() && b_move_cmd) {
            //     std::cout << "First Moving Command Received" << std::endl;
            //     ((ScorpioInterface*)interface_)->MoveEndEffectorTo(-0.5, 0.1, 1.3, 0.000316,-0.7071,0.00025, 0.7071); //Maintain initial orientation
            //     b_move_cmd = false;
            // }
            //
            // static bool b_grasp_cmd(true);
            // if (((ScorpioInterface*)interface_)->IsReadyToGrasp() && b_grasp_cmd) {
            //     std::cout << "First Grasping Command Received" << std::endl;
            //     ((ScorpioInterface*)interface_)->Grasp();
            //     b_grasp_cmd = false;
            // }
            //
            // static bool b_move_while_hold_cmd(true);
            // if (((ScorpioInterface*)interface_)->IsReadyToMove() && b_move_while_hold_cmd) {
            //     std::cout << "First Moving While Holding Command Received" << std::endl;
            //     ((ScorpioInterface*)interface_)->MoveEndEffectorTo(-0.3, -0.5, 1.5, 0.000316,-0.7071,0.00025, 0.7071);
            //     b_move_while_hold_cmd = false;
            // }
            //
            // static bool b_release_cmd(true);
            // if (((ScorpioInterface*)interface_)->IsReadyToMove() && b_release_cmd) {
            //     std::cout << "First Pouring Command Received" << std::endl;
            //     ((ScorpioInterface*)interface_)->MoveEndEffectorTo(-0.3, -0.5, 1.5,-0.7070, 0, 0, 0.7073);
            //     b_release_cmd = false;
            // }

            sensordata_->q = q_;
            sensordata_->qdot = qdot_;
            interface_->getCommand(sensordata_, command_);

            for (int i = 0; i < active_joint_idx_.size(); ++i) {
                joint_msg_.position.push_back((sensordata_->q_act)[i]);
            }
            endeff_msg_.position.x = (interface_->endeff_pos_)[0];
            endeff_msg_.position.y = (interface_->endeff_pos_)[1];
            endeff_msg_.position.z = (interface_->endeff_pos_)[2];
            endeff_msg_.orientation.x = (interface_->endeff_ori_).x();
            endeff_msg_.orientation.y = (interface_->endeff_ori_).y();
            endeff_msg_.orientation.z = (interface_->endeff_ori_).z();
            endeff_msg_.orientation.w = (interface_->endeff_ori_).w();

            jointPub_.publish(joint_msg_);
            endeffPub_.publish(endeff_msg_);

            // Clear joint_msg_ or position array will continuously build up
            joint_msg_.position.clear();

            for (int i = 0; i < active_joint_idx_.size(); ++i) {
                joints[active_joint_idx_[i]]->SetForce(0, command_->jtrq[i]);
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

        ScorpioInterface *interface_;
        ScorpioSensorData *sensordata_;
        ScorpioCommand *command_;

        //Ros stuff
        std::unique_ptr <ros::NodeHandle> rosNode_;
        ros::Subscriber rosSub_;
        ros::Publisher jointPub_;
        ros::Publisher endeffPub_;
        ros::ServiceServer endeffServer_;
        ros::ServiceServer gripperServer_;
        ros::ServiceServer interruptServer_;
        sensor_msgs::JointState joint_msg_;
        geometry_msgs::Pose endeff_msg_;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ScorpioPlugin)
}
