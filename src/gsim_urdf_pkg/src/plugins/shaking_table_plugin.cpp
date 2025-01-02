#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <vector>
#include <string>

namespace gazebo {
class ShakingTablePlugin : public ModelPlugin {
public:
  ShakingTablePlugin() : frequency(1.0), amplitude(0.01), timeElapsed(0.0) {}

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override {
    this->model = _parent;

    // Parse parameters from SDF/URDF
    if (_sdf->HasElement("amplitude")) {
      this->amplitude = _sdf->Get<double>("amplitude");
    }
    if (_sdf->HasElement("frequency_min")) {
      this->frequency_min = _sdf->Get<double>("frequency_min");
    }
    if (_sdf->HasElement("frequency_max")) {
      this->frequency_max = _sdf->Get<double>("frequency_max");
    }
    if (_sdf->HasElement("sweep_rate")) {
      this->sweep_rate = _sdf->Get<double>("sweep_rate");
    }

    // Define natural frequencies for each floor
    this->naturalFrequencies = {2.0, 4.0, 6.0, 8.0, 10.0}; // Example values

    // Retrieve joints
    for (auto jointName : {"Joint_First", "Joint_Second", "Joint_Third", "Joint_Fourth", "Joint_Fifth"}) {
      auto joint = this->model->GetJoint(jointName);
      if (joint) {
        this->joints.push_back(joint);
      } else {
        gzerr << "Joint '" << jointName << "' not found. Check your URDF." << std::endl;
      }
    }

    // Initialize ROS
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "shaking_table_plugin",
                ros::init_options::NoSigintHandler);
    }
    this->rosNode.reset(new ros::NodeHandle("shaking_table_plugin"));

    // Publisher for frequency feedback
    this->frequencyPub = this->rosNode->advertise<std_msgs::Float64>(
        "/shaking_table/frequency", 1);

    // Connect to the world update event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ShakingTablePlugin::OnUpdate, this));

    gzlog << "ShakingTablePlugin loaded successfully!" << std::endl;
  }

  void OnUpdate() {
    // Increment time
    this->timeElapsed += this->model->GetWorld()->Physics()->GetMaxStepSize();

    // Update frequency with a sweep signal
    this->frequency = this->frequency_min + this->timeElapsed * this->sweep_rate;
    if (this->frequency > this->frequency_max) {
      this->frequency = this->frequency_max;
    }

    // Calculate base displacement
    double baseDisplacement = this->amplitude * sin(2 * M_PI * this->frequency * this->timeElapsed);

    // Apply motion to each joint
    for (size_t i = 0; i < this->joints.size(); ++i) {
      double floorDisplacement = baseDisplacement;

      // Check for resonance
      if (std::abs(this->frequency - this->naturalFrequencies[i]) < 0.1) {
        floorDisplacement += 0.002; // Add extra displacement during resonance
      }

      // Set joint position
      this->joints[i]->SetPosition(0, floorDisplacement);
    }

    // Publish the current frequency
    std_msgs::Float64 freqMsg;
    freqMsg.data = this->frequency;
    this->frequencyPub.publish(freqMsg);
  }

private:
  physics::ModelPtr model;
  std::vector<physics::JointPtr> joints;
  event::ConnectionPtr updateConnection;
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Publisher frequencyPub;

  double frequency;
  double frequency_min = 1.0;
  double frequency_max = 30.0;
  double sweep_rate = 0.05;
  double amplitude;
  double timeElapsed;
  std::vector<double> naturalFrequencies;
};

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(ShakingTablePlugin)
} // namespace gazebo
