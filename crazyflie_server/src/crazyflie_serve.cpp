#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

// The C++ library from your code snippet:
#include "Crazyflie.h"

// We can use LogBlockGeneric to handle variable-sized logs:
#include <vector>
#include <string>
#include <functional>
#include <memory>
#include <cmath>

class CrazyflieServerNode
{
public:
  CrazyflieServerNode()
  {
    ros::NodeHandle nh("~");

    // -------------------------------
    // 1) Read ROS parameters
    // -------------------------------
    // Example booleans for logging
    velLOG_      = nh.param<bool>("vel_LOG", true);
    angLOG_      = nh.param<bool>("ang_LOG", true);
    thrustLOG_   = nh.param<bool>("thrust_LOG", true);
    accLOG_      = nh.param<bool>("acc_LOG", true);
    gyroRawLOG_  = nh.param<bool>("gyro_raw_LOG", true);
    gyroLOG_     = nh.param<bool>("gyro_LOG", true);
    useBodyRate_ = nh.param<bool>("use_body_rate", false);

    stabilizer_controller_ = nh.param<int>("stabilizer_controller", 1);
    stabilizer_estimator_  = nh.param<int>("stabilizer_estimator", 3);

    // ID -> URI
    int ID = -1;
    if (!nh.getParam("ID", ID)) {
      ROS_ERROR("You must set the '~ID' parameter!");
      throw std::runtime_error("Missing ~ID param");
    }
    std::string uri = mapIDToUri(ID);

    // -------------------------------
    // 2) Create Crazyflie object
    // -------------------------------
    // The constructor automatically opens the link (radio://...).
    // Third argument is an optional console callback.
    cf_ = std::make_shared<Crazyflie>(uri, EmptyLogger,
      [this](const char* msg){
        ROS_INFO("[CF Console] %s", msg);
      }
    );

    // (Optional) explicitly request param/log TOCs so we can set/read parameters
    try {
      cf_->requestParamToc(/*forceNoCache=*/false);
      cf_->requestLogToc(/*forceNoCache=*/false);
    } catch (std::exception& e) {
      ROS_ERROR("Failed to request param/log TOC: %s", e.what());
    }

    // -------------------------------
    // 3) Set initial parameters
    // -------------------------------
    // In Python:  _cf.param.set_value("stabilizer.controller", stabilizer_controller)
    // In C++:     cf_->setParamByName("stabilizer", "controller", floatValue)
    try {
      cf_->setParamByName("stabilizer", "controller", (float)stabilizer_controller_);
      ROS_INFO("Set stabilizer.controller to %d", stabilizer_controller_);
    } catch (std::exception &e) {
      ROS_ERROR("Failed setting stabilizer.controller: %s", e.what());
    }

    try {
      cf_->setParamByName("stabilizer", "estimator", (float)stabilizer_estimator_);
      ROS_INFO("Set stabilizer.estimator to %d", stabilizer_estimator_);
    } catch (std::exception &e) {
      ROS_ERROR("Failed setting stabilizer.estimator: %s", e.what());
    }

    // Switch flight mode
    if (useBodyRate_) {
      ROS_INFO("Using Body Rate mode");
      try {
        cf_->setParamByName("flightmode", "stabModeRoll", 0.0f);
        cf_->setParamByName("flightmode", "stabModePitch", 0.0f);
        cf_->setParamByName("flightmode", "stabModeYaw", 0.0f);
      } catch (std::exception &e) {
        ROS_ERROR("Failed setting flightmode.stabMode*: %s", e.what());
      }
    } else {
      ROS_INFO("Using Angle mode");
      try {
        cf_->setParamByName("flightmode", "stabModeRoll", 1.0f);
        cf_->setParamByName("flightmode", "stabModePitch", 1.0f);
        cf_->setParamByName("flightmode", "stabModeYaw", 1.0f);
      } catch (std::exception &e) {
        ROS_ERROR("Failed setting flightmode.stabMode*: %s", e.what());
      }
    }

    // Unlock startup thrust protection if needed
    cf_->sendSetpoint(0.0f, 0.0f, 0.0f, 0);

    // -------------------------------
    // 4) Initialize ROS pubs/subs
    // -------------------------------
    // Subscriber for cmd_vel -> setpoint
    subCmdVel_ = nh.subscribe("cmd_vel", 1, &CrazyflieServerNode::cmdVelCallback, this);

    // Basic publishers (matching your original Python code)
    if (velLOG_)      pubVel_      = nh.advertise<geometry_msgs::Vector3Stamped>("crazyflieVel", 10);
    if (angLOG_)      pubAng_      = nh.advertise<geometry_msgs::Vector3Stamped>("crazyflieAng", 10);
    if (thrustLOG_)   pubThrust_   = nh.advertise<geometry_msgs::Vector3Stamped>("crazyflieThrust", 10);
    if (accLOG_)      pubAcc_      = nh.advertise<geometry_msgs::Vector3Stamped>("crazyflieAcc", 10);
    if (gyroRawLOG_)  pubRawGyro_  = nh.advertise<geometry_msgs::Vector3Stamped>("crazyflieRawAngRate", 10);
    if (gyroLOG_)     pubGyro_     = nh.advertise<geometry_msgs::Vector3Stamped>("crazyflieAngRate", 10);

    pubBatteryVoltage_ = nh.advertise<std_msgs::Float32>("crazyflieBatteryVoltage", 10);
    pubBatteryLevel_   = nh.advertise<std_msgs::Float32>("crazyflieBatteryLevel", 10);
    pubIsFlying_       = nh.advertise<std_msgs::Bool>("crazyflieIsFlying", 10);
    pubCanFly_         = nh.advertise<std_msgs::Bool>("crazyflieCanFly", 10);
    pubZRange_         = nh.advertise<geometry_msgs::Vector3Stamped>("crazyflieZRange", 10);

    // -------------------------------
    // 5) Example: set up a LogBlock for battery
    // -------------------------------
    // We want to log "pm.vbat" and "pm.batteryLevel" at, e.g., 1 Hz => period=100 (ms * 10?), or 10 Hz => period=100/10=10 ...
    // The library uses "period" in increments of 10ms. A "period" of 10 => 100ms => 10Hz
    // We'll demonstrate "LogBlockGeneric", which can handle multiple variables in one block.
    if (true) // or your battery_flag if you want
    {
      std::vector<std::string> variables;
      variables.push_back("pm.vbat");
      variables.push_back("pm.batteryLevel");

      // We store "this" as userData so we can publish from the callback
      auto callback = [](uint32_t time_in_ms, std::vector<double>* values, void* userData)
      {
        // userData is our CrazyflieServerNode*
        auto self = reinterpret_cast<CrazyflieServerNode*>(userData);
        if (values->size() < 2) {
          // Something unexpected
          return;
        }
        float batteryVoltage = (float)(values->at(0));
        float batteryLevel   = (float)(values->at(1));

        // Publish
        std_msgs::Float32 msgV;
        msgV.data = batteryVoltage;
        self->pubBatteryVoltage_.publish(msgV);

        std_msgs::Float32 msgL;
        msgL.data = batteryLevel;
        self->pubBatteryLevel_.publish(msgL);
      };

      // Make a log block
      batteryLogBlock_ = std::make_shared<LogBlockGeneric>(
          cf_.get(),
          variables,
          /* userData = */ this,
          /* callback = */ callback);

      // Start the log block with period=10 => 100ms => 10Hz
      // If you want 1Hz, use period=100 => 1s
      batteryLogBlock_->start(/* period = */ 10);
      ROS_INFO("Started battery log block.");
    }

    // You can create additional LogBlockGeneric instances for velocity, angles, accelerations, etc.
    // See the bottom of this file for more info on how to add more logs.

    ROS_INFO("CrazyflieServerNode C++: init done.");
  }

  ~CrazyflieServerNode()
  {
    // Clean up log blocks so they stop logging
    if (batteryLogBlock_) {
      batteryLogBlock_->stop();
    }
    // If you made others, stop them similarly

    // Send a “stop” setpoint so the CF stops spinning
    if (cf_) {
      cf_->sendStop();
    }
  }

private:
  // -------------------------------
  // 6) cmd_vel callback
  // -------------------------------
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    // Convert from Twist -> Crazyflie setpoint
    // Python used deg, so we replicate that:
    constexpr float RAD2DEG = 57.2958f;
    int thrust = static_cast<int>(msg->linear.z);

    if (useBodyRate_) {
      float rollrateDeg   = msg->angular.x * RAD2DEG;
      float pitchrateDeg  = msg->angular.y * RAD2DEG;
      float yawrateDeg    = msg->angular.z * RAD2DEG;

      // Negative yaw rate to match your Python code’s sign
      cf_->sendSetpoint(rollrateDeg, pitchrateDeg, -yawrateDeg, thrust);
    } else {
      float rollDeg  =  msg->angular.x * RAD2DEG;
      float pitchDeg =  msg->angular.y * RAD2DEG;
      float yawrate  =  msg->angular.z * RAD2DEG;

      // The Python code also does negative roll and negative yaw
      cf_->sendSetpoint(-rollDeg, pitchDeg, -yawrate, thrust);
    }
  }

  // Example function to map ID -> URI (like your Python code)
  std::string mapIDToUri(int ID)
  {
    switch (ID) {
      case 1: return "radio://0/10/2M/E7E7E7E701";
      case 2: return "radio://0/10/2M/E7E7E7E702";
      case 3: return "radio://0/10/2M/E7E7E7E703";
      case 4: return "radio://0/10/2M/E7E7E7E704";
      case 5: return "radio://0/20/2M/E7E7E7E705";
      case 6: return "radio://0/20/2M/E7E7E7E706";
      case 7: return "radio://0/20/2M/E7E7E7E707";
      case 8: return "radio://0/50/2M/E7E7E7E708";
    }
    throw std::runtime_error("Invalid ID param: must be 1..8");
  }

private:
  // The main Crazyflie object
  std::shared_ptr<Crazyflie> cf_;

  // A sample log block (battery). You can create more for velocity, angle, etc.
  std::shared_ptr<LogBlockGeneric> batteryLogBlock_;

  // ROS Subscribers
  ros::Subscriber subCmdVel_;

  // ROS Publishers
  ros::Publisher pubVel_;
  ros::Publisher pubAng_;
  ros::Publisher pubThrust_;
  ros::Publisher pubAcc_;
  ros::Publisher pubRawGyro_;
  ros::Publisher pubGyro_;
  ros::Publisher pubBatteryVoltage_;
  ros::Publisher pubBatteryLevel_;
  ros::Publisher pubIsFlying_;
  ros::Publisher pubCanFly_;
  ros::Publisher pubZRange_;

  // ROS params / flags
  bool velLOG_;
  bool angLOG_;
  bool thrustLOG_;
  bool accLOG_;
  bool gyroRawLOG_;
  bool gyroLOG_;
  bool useBodyRate_;
  int stabilizer_controller_;
  int stabilizer_estimator_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crazyflie_server_node_cpp");
  try {
    CrazyflieServerNode node;
    ros::spin();
  } catch (std::exception &e) {
    ROS_ERROR("Unhandled exception in CrazyflieServerNode: %s", e.what());
  }
  return 0;
}
