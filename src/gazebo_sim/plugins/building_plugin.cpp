#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

namespace gazebo
{
  class BuildingPlugin : public ModelPlugin
  {
    private:
      physics::ModelPtr model;
      ros::NodeHandle* rosNode;
      ros::Subscriber scaleSub;

    public:
      void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
      {
        this->model = _model;

        // Initialize ROS node
        if (!ros::isInitialized())
        {
          int argc = 0;
          char **argv = nullptr;
          ros::init(argc, argv, "building_plugin");
        }
        this->rosNode = new ros::NodeHandle("building_plugin");

        // Subscribe to the scale topic
        this->scaleSub = this->rosNode->subscribe<std_msgs::Float32MultiArray>(
            "/building/scale", 1, &BuildingPlugin::OnScaleMsg, this);
      }

      void OnScaleMsg(const std_msgs::Float32MultiArray::ConstPtr& msg)
      {
        if (msg->data.size() == 3)
        {
          ignition::math::Vector3d scale(msg->data[0], msg->data[1], msg->data[2]);
          this->model->SetScale(scale);
        }
        else
        {
          gzerr << "Scale message must have 3 values for X, Y, Z scaling.\n";
        }
      }
  };

  GZ_REGISTER_MODEL_PLUGIN(BuildingPlugin)
}
