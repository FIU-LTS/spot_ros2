#include <cmath>  // For std::abs
#include <memory>
#include <string>
#include <vector>
#include <chrono> // For std::chrono literals

#include "geometry_msgs/msg/pose.hpp"  // For publishing
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp" // For /stand service
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // For tf2::toMsg
#include "yaml-cpp/yaml.h"

// Configuration structure for body pose control
struct BodyPoseControlConfig {
  bool enabled = false;
  int teleop_main_enable_button_idx = -1;
  int axis_body_roll = -1;
  int axis_body_pitch = -1;
  double scale_body_roll = 0.0;
  double scale_body_pitch = 0.0;
  double deadzone = 0.0;
  std::string body_pose_topic_name;
  double stand_call_interval_sec = 1.0; 
};

class JoyBodyPoseControllerNode : public rclcpp::Node {
 public:
  JoyBodyPoseControllerNode() : Node("joy_body_pose_controller"),
                                prev_roll_input_val_(0.0), 
                                prev_pitch_input_val_(0.0),
                                // Initialize rclcpp::Time and rclcpp::Duration members here
                                // Use a valid constructor. They will be properly set after load_config.
                                last_stand_call_attempt_time_(this->get_clock()->now()), // Initialize with current time
                                stand_call_min_interval_(0, 0) { // Initialize with zero duration
    this->declare_parameter<std::string>("config_file_path", "joy_body_pose_config.yaml");
    this->declare_parameter<std::string>("spot_name", ""); 

    std::string config_path = this->get_parameter("config_file_path").as_string();
    spot_name_ = this->get_parameter("spot_name").as_string();

    load_config(config_path); 

    // Now properly set them based on config_ after it's loaded
    last_stand_call_attempt_time_ = this->get_clock()->now() - rclcpp::Duration::from_seconds(config_.stand_call_interval_sec + 1.0);
    stand_call_min_interval_ = rclcpp::Duration::from_seconds(config_.stand_call_interval_sec);

    if (config_.enabled) {
      joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
          "/joy", 10, std::bind(&JoyBodyPoseControllerNode::joy_callback, this, std::placeholders::_1));

      body_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(config_.body_pose_topic_name, 10);

      std::string stand_service_name = "stand";
      if (!spot_name_.empty()) {
        stand_service_name = "/" + spot_name_ + "/stand";
      }
      stand_service_client_ = this->create_client<std_srvs::srv::Trigger>(stand_service_name);

      body_pose_update_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),  // 10 Hz
                                                        std::bind(&JoyBodyPoseControllerNode::timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "'%s' started and body pose control is enabled.", this->get_name());
      RCLCPP_INFO(this->get_logger(), "  Spot Name: '%s'", spot_name_.c_str());
      RCLCPP_INFO(this->get_logger(), "  Stand Service: '%s'", stand_service_name.c_str());
      RCLCPP_INFO(this->get_logger(), "  Stand Call Interval: %.2f s", config_.stand_call_interval_sec);
      RCLCPP_INFO(this->get_logger(), "  Teleop Enable Button Idx: %d", config_.teleop_main_enable_button_idx);
      RCLCPP_INFO(this->get_logger(), "  Axis Roll: %d, Axis Pitch: %d", config_.axis_body_roll,
                  config_.axis_body_pitch);
      RCLCPP_INFO(this->get_logger(), "  Scale Roll: %.2f, Scale Pitch: %.2f", config_.scale_body_roll,
                  config_.scale_body_pitch);
      RCLCPP_INFO(this->get_logger(), "  Publishing to topic: %s", config_.body_pose_topic_name.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "'%s' started, but body pose control is disabled in the configuration.",
                  this->get_name());
    }
  }

 private:
  void load_config(const std::string& file_path) {
    if (file_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Config file path parameter is empty. Body pose control disabled.");
      config_.enabled = false;
      return;
    }
    try {
      YAML::Node config_root = YAML::LoadFile(file_path);
      std::string node_name_str = this->get_name();

      if (config_root[node_name_str] && config_root[node_name_str]["ros__parameters"]) {
        YAML::Node params = config_root[node_name_str]["ros__parameters"];
        config_.enabled = params["enabled"] ? params["enabled"].as<bool>() : false;
        if (config_.enabled) {
          config_.teleop_main_enable_button_idx =
              params["teleop_main_enable_button_idx"] ? params["teleop_main_enable_button_idx"].as<int>() : -1;
          config_.axis_body_roll = params["axis_body_roll"] ? params["axis_body_roll"].as<int>() : -1;
          config_.axis_body_pitch = params["axis_body_pitch"] ? params["axis_body_pitch"].as<int>() : -1;
          config_.scale_body_roll = params["scale_body_roll"] ? params["scale_body_roll"].as<double>() : 0.0;
          config_.scale_body_pitch = params["scale_body_pitch"] ? params["scale_body_pitch"].as<double>() : 0.0;
          config_.deadzone = params["deadzone"] ? params["deadzone"].as<double>() : 0.0;
          config_.body_pose_topic_name =
              params["body_pose_topic_name"] ? params["body_pose_topic_name"].as<std::string>() : "";
          config_.stand_call_interval_sec =
              params["stand_call_interval_sec"] ? params["stand_call_interval_sec"].as<double>() : 1.0;

          if (config_.teleop_main_enable_button_idx == -1 ||
              config_.body_pose_topic_name.empty() || config_.axis_body_roll == -1 || config_.axis_body_pitch == -1) {
            RCLCPP_ERROR(this->get_logger(),
                         "Body pose control is enabled but critical parameters "
                         "('teleop_main_enable_button_idx', 'body_pose_topic_name', "
                         "'axis_body_roll', 'axis_body_pitch') "
                         "are missing/invalid in config. Disabling feature.");
            config_.enabled = false;
          }
        }
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "Configuration section for '%s.ros__parameters' not found in YAML: %s. Body pose control disabled.",
                    node_name_str.c_str(), file_path.c_str());
        config_.enabled = false;
      }
    } catch (const YAML::BadFile& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to load body pose config file (BadFile): '%s'. Error: %s. Body pose control disabled.",
                   file_path.c_str(), e.what());
      config_.enabled = false;
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "YAML parsing error in body pose config '%s': %s. Body pose control disabled.",
                   file_path.c_str(), e.what());
      config_.enabled = false;
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    last_joy_msg_ = joy_msg;
  }

  void call_stand_service() {
    if (stand_service_call_in_progress_) {
        RCLCPP_DEBUG(this->get_logger(), "Stand service call already in progress. Skipping.");
        return;
    }
    if (!stand_service_client_ || !stand_service_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Stand service not available or not ready.");
      return;
    }

    stand_service_call_in_progress_ = true;
    last_stand_call_attempt_time_ = this->get_clock()->now(); 

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    stand_service_client_->async_send_request(request, 
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            stand_service_call_in_progress_ = false; 
            try {
                auto response = future.get(); 
                if (response && response->success) {
                    RCLCPP_DEBUG(this->get_logger(), "Stand service call successful: %s", response->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Stand service call failed or service returned failure: %s", 
                                (response ? response->message.c_str() : "N/A - Service call future did not complete successfully or no response"));
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during stand service call future.get(): %s", e.what());
            }
        });
  }

  void timer_callback() {
    if (!config_.enabled || !last_joy_msg_) {
      return;
    }

    bool main_teleop_button_active = false;
    if (config_.teleop_main_enable_button_idx != -1 &&
        static_cast<size_t>(config_.teleop_main_enable_button_idx) < last_joy_msg_->buttons.size()) {
      main_teleop_button_active = (last_joy_msg_->buttons[config_.teleop_main_enable_button_idx] == 1);
    }

    if (main_teleop_button_active) {
      if (!neutral_pose_sent_since_teleop_active_) {
        RCLCPP_DEBUG(this->get_logger(), "Timer: Main teleop ACTIVE. Sending neutral body pose.");
        publish_body_pose_command(0.0, 0.0);
        neutral_pose_sent_since_teleop_active_ = true;
      }
      // Reset previous inputs when teleop is active so a stand command is triggered on next axis move
      prev_roll_input_val_ = 0.0; 
      prev_pitch_input_val_ = 0.0;
    } else {
      neutral_pose_sent_since_teleop_active_ = false;

      double current_roll_input_val = 0.0;
      double current_pitch_input_val = 0.0;

      if (config_.axis_body_roll != -1 && static_cast<size_t>(config_.axis_body_roll) < last_joy_msg_->axes.size()) {
        current_roll_input_val = last_joy_msg_->axes[config_.axis_body_roll];
      }
      if (config_.axis_body_pitch != -1 && static_cast<size_t>(config_.axis_body_pitch) < last_joy_msg_->axes.size()) {
        current_pitch_input_val = last_joy_msg_->axes[config_.axis_body_pitch];
      }

      if (std::abs(current_roll_input_val) < config_.deadzone) current_roll_input_val = 0.0;
      if (std::abs(current_pitch_input_val) < config_.deadzone) current_pitch_input_val = 0.0;

      // Check for significant change in axis input
      // Using a small epsilon for floating point comparison, or rely on deadzone effect
      bool axis_changed = (std::abs(current_roll_input_val - prev_roll_input_val_) > 1e-5) ||
                          (std::abs(current_pitch_input_val - prev_pitch_input_val_) > 1e-5);

      if (axis_changed) {
        RCLCPP_DEBUG(this->get_logger(), "Axis input changed. Prev (R:%.2f, P:%.2f), Curr (R:%.2f, P:%.2f)",
            prev_roll_input_val_, prev_pitch_input_val_, current_roll_input_val, current_pitch_input_val);
        // Rate-limit the stand service call only if axis changed
        if (!stand_service_call_in_progress_ && 
            (this->get_clock()->now() - last_stand_call_attempt_time_ > stand_call_min_interval_)) {
          RCLCPP_INFO(this->get_logger(), "Attempting to call /stand due to axis change.");
          call_stand_service();
        }
      }

      double target_roll = current_roll_input_val * config_.scale_body_roll;
      double target_pitch = current_pitch_input_val * config_.scale_body_pitch;

      RCLCPP_DEBUG(this->get_logger(), "Timer: Main teleop DISENGAGED. Publishing body pose (R=%.2f, P=%.2f).",
                  target_roll, target_pitch);
      publish_body_pose_command(target_roll, target_pitch);

      // Update previous values for next iteration
      prev_roll_input_val_ = current_roll_input_val;
      prev_pitch_input_val_ = current_pitch_input_val;
    }
  }

  void publish_body_pose_command(double roll, double pitch, double yaw = 0.0, double height_z = 0.0) {
    if (!body_pose_publisher_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Body pose publisher not initialized.");
      return;
    }

    auto pose_msg = std::make_unique<geometry_msgs::msg::Pose>();
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    pose_msg->orientation = tf2::toMsg(q);
    pose_msg->position.x = 0.0;
    pose_msg->position.y = 0.0;
    pose_msg->position.z = height_z;

    RCLCPP_DEBUG(this->get_logger(), "Publishing BodyPose: R=%.2f, P=%.2f, Y=%.2f, Z=%.2f to %s", roll, pitch, yaw,
                 height_z, config_.body_pose_topic_name.c_str());
    body_pose_publisher_->publish(std::move(pose_msg));
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  BodyPoseControlConfig config_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr body_pose_publisher_;
  rclcpp::TimerBase::SharedPtr body_pose_update_timer_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;
  bool neutral_pose_sent_since_teleop_active_ = false;
  
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stand_service_client_;
  std::string spot_name_;
  rclcpp::Time last_stand_call_attempt_time_;
  rclcpp::Duration stand_call_min_interval_;
  bool stand_service_call_in_progress_ = false;

  // Store previous axis values to detect change
  double prev_roll_input_val_;
  double prev_pitch_input_val_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyBodyPoseControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
