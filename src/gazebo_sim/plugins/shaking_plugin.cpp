#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
  class ShakingPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      this->model = _parent;

      // Initialize ROS node handle
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "shaking_plugin");
      }
      this->nh.reset(new ros::NodeHandle("shaking_plugin"));

      // Subscribe to topics for amplitude and frequency
      this->amplitude_sub = this->nh->subscribe<std_msgs::Float64>("/shaking_table/amplitude", 1, &ShakingPlugin::SetAmplitude, this);
      this->frequency_sub = this->nh->subscribe<std_msgs::Float64>("/shaking_table/frequency", 1, &ShakingPlugin::SetFrequency, this);

      // Default values
      this->amplitude = _sdf->Get<double>("amplitude");
      this->frequency = _sdf->Get<double>("frequency");

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ShakingPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      double time = this->model->GetWorld()->SimTime().Double();
      double displacement = this->amplitude * sin(2 * M_PI * this->frequency * time);
      this->model->SetLinearVel(ignition::math::Vector3d(displacement, 0, 0));
    }

    void SetAmplitude(const std_msgs::Float64::ConstPtr &msg)
    {
      this->amplitude = msg->data;
    }

    void SetFrequency(const std_msgs::Float64::ConstPtr &msg)
    {
      this->frequency = msg->data;
    }

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Subscriber amplitude_sub, frequency_sub;
    double amplitude, frequency;
  };

  GZ_REGISTER_MODEL_PLUGIN(ShakingPlugin)
}
