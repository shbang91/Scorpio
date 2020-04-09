#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
//#include "gazebo/physics/dart/DARTPhysics.hh"
//#include "gazebo/physics/dart/DARTModel.hh"
//#include <dart/dart.hpp>
//#include <dart/gui/osg/osg.hpp>
//#include <dart/utils/urdf/urdf.hpp>
//#include <dart/utils/utils.hpp>

namespace gazebo
{
    class ScorpioPlugin : public ModelPlugin
    {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&ScorpioPlugin::OnUpdate, this));

            //physics::DARTModel *m = (physics::DARTModel *)(model.get());
            //dart::utils::DartLoader urdfLoade
            //dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton("/home/will/workspace/PnC/RobotModel/Robot/Draco/DracoSim_Dart.urdf");
            //m->DARTSkeleton() = robot;
        }

        // Called by the world update start event
    public: void OnUpdate()
        {
            // Apply a small linear velocity to the model.
            //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
        }

        // Pointer to the model
    private: physics::ModelPtr model;

        // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ScorpioPlugin)
}
