#include <functional>
#include <ignition/math.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "worldsim/Act_msg.h"

#define WALKING_ANIMATION "walking"

namespace gazebo
{
  class ActorPlugin : public ModelPlugin
  {
    /// Constructor
    public: ActorPlugin() {}

    //Load Fn
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->sdf = _sdf;
      this->model = _model;
      this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
      this->world = this->actor->GetWorld();

      this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
              std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

      this->Reset();

      // Read in the target weight
      if (_sdf->HasElement("target_weight"))
        this->targetWeight = _sdf->Get<double>("target_weight");
      else
        this->targetWeight = 1.15;

      // Read in the obstacle weight
      if (_sdf->HasElement("obstacle_weight"))
        this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
      else
        this->obstacleWeight = 1.5;

      // Read in the animation factor (applied in the OnUpdate function).
      if (_sdf->HasElement("animation_factor"))
        this->animationFactor = _sdf->Get<double>("animation_factor");
      else
        this->animationFactor = 4.5;

      // Add our own name to models we should ignore when avoiding obstacles.
      this->ignoreModels.push_back(this->actor->GetName());

      // Read in the other obstacles to ignore
      if (_sdf->HasElement("ignore_obstacles"))
      {
        sdf::ElementPtr modelElem =
          _sdf->GetElement("ignore_obstacles")->GetElement("model");
        while (modelElem)
        {
          this->ignoreModels.push_back(modelElem->Get<std::string>());
          modelElem = modelElem->GetNextElement("model");
        }
      }

      /// ---------------- ROS ---------------- ///
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<worldsim::Act_msg>(
            "/" + this->model->GetName() + "/vel_cmd",   
            1,
            boost::bind(&ActorPlugin::OnRosMsg, this, _1), 
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ActorPlugin::QueueThread, this));

    }

    /////////////////////////////////////////////////
    public: virtual void Reset()
    {
      // Default velocity 
      this->velocity = 0.8;
      this->lastUpdate = 0;

      if (this->sdf && this->sdf->HasElement("velocity"))
        this->velocity = this->sdf->Get<double>("velocity");
      
      if (this->sdf && this->sdf->HasElement("target"))
        this->target = this->sdf->Get<ignition::math::Vector3d>("target");
      else
        this->target = ignition::math::Vector3d(0, -5, 1.2138);

      auto skelAnims = this->actor->SkeletonAnimations();
      if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
      {
        gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
      }
      else
      {
        // Create custom trajectory
        this->trajectoryInfo.reset(new physics::TrajectoryInfo());
        this->trajectoryInfo->type = WALKING_ANIMATION;
        this->trajectoryInfo->duration = 1.0;

        this->actor->SetCustomTrajectory(this->trajectoryInfo);
      }
    }

    /////////////////////////////////////////////////
    private: void OnUpdate(const common::UpdateInfo &_info)
    {
      // Time delta
      double dt = (_info.simTime - this->lastUpdate).Double();

      ignition::math::Pose3d pose = this->actor->WorldPose();
      ignition::math::Vector3d pos = this->target - pose.Pos();
      ignition::math::Vector3d rpy = pose.Rot().Euler();

      double distance = pos.Length();

      // Normalize the direction vector, and apply the target weight
      pos = pos.Normalize() * this->targetWeight;

      // Compute the yaw orientation
      ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
      yaw.Normalize();

      // Rotate in place, instead of jumping.
      if (std::abs(yaw.Radian()) > IGN_DTOR(10))
      {
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
            yaw.Radian()*0.001);
      }
      else
      {
        pose.Pos() += pos * this->velocity * dt;
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
      }

      // Make sure the actor stays within bounds
      // pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
      // pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
      // pose.Pos().Z(1.2138);

      // Distance traveled is used to coordinate motion with the walking
      // animation
      double distanceTraveled = (pose.Pos() -
          this->actor->WorldPose().Pos()).Length();

      this->actor->SetWorldPose(pose, false, false);
      this->actor->SetScriptTime(this->actor->ScriptTime() +
        (distanceTraveled * this->animationFactor));
      this->lastUpdate = _info.simTime;
    }

    /// ---------------- ROS ---------------- ///
    public: void OnRosMsg(const worldsim::Act_msg::ConstPtr &_msg)
    {
      //float vel = msg->velocity;
      //this->velocity = vel;
      this->velocity = _msg->velocity;
      this->target = ignition::math::Vector3d(_msg->target_x, _msg->target_y, 1.2138);
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }


    /// ============================ Variables ============================ ///
    /// ---------------- ROS ---------------- ///
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    /// -------------- End-ROS--------------- ///

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;
    
    /// \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    /// \brief Velocity of the actor
    private: ignition::math::Vector3d velocity;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
    private: ignition::math::Vector3d target;

    /// \brief Target location weight (used for vector field)
    private: double targetWeight = 1.0;

    /// \brief Obstacle weight (used for vector field)
    private: double obstacleWeight = 1.0;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    private: double animationFactor = 1.0;

    /// \brief Time of the last update.
    private: common::Time lastUpdate;

    /// \brief List of models to ignore. Used for vector field
    private: std::vector<std::string> ignoreModels;

    /// \brief Custom trajectory info.
    private: physics::TrajectoryInfoPtr trajectoryInfo;

  };
  GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)
}

