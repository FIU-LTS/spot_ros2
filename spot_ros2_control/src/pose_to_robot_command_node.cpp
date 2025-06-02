#include <memory>
#include <string>
#include <chrono>
#include <cmath> // For std::abs
#include <vector> // For std::vector<uint8_t>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "spot_msgs/action/robot_command.hpp" // Correct: This is the action definition itself

// Includes for the command structure, based on ros2 interface show
#include "bosdyn_api_msgs/msg/robot_command.hpp" // Defines RobotCommand and its nested RobotCommandOneOfCommand
#include "bosdyn_api_msgs/msg/synchronized_command_request.hpp"
#include "bosdyn_api_msgs/msg/mobility_command_request.hpp"
#include "proto2ros/msg/any_proto.hpp" // For the 'params' field in MobilityCommandRequest

// Geometry types are correctly from bosdyn_api_msgs
#include "bosdyn_api_msgs/msg/se3_trajectory.hpp"
#include "bosdyn_api_msgs/msg/se3_trajectory_point.hpp"

// Protobuf includes
#include <bosdyn/api/spot/robot_command.pb.h> // For spot::MobilityParams, spot::BodyControlParams
#include <bosdyn/api/geometry.pb.h>           // For geometry::SE3Trajectory, geometry::SE3TrajectoryPoint protos
#include <google/protobuf/any.pb.h>           // For google::protobuf::Any
#include <google/protobuf/duration.pb.h> // For time_since_reference in proto

// Helper to convert geometry_msgs::Pose to bosdyn::api::SE3Pose (protobuf)
void convertRosPoseToProtoSE3Pose(const geometry_msgs::msg::Pose& ros_pose, bosdyn::api::SE3Pose& proto_pose) {
    proto_pose.mutable_position()->set_x(ros_pose.position.x);
    proto_pose.mutable_position()->set_y(ros_pose.position.y);
    proto_pose.mutable_position()->set_z(ros_pose.position.z);
    proto_pose.mutable_rotation()->set_w(ros_pose.orientation.w);
    proto_pose.mutable_rotation()->set_x(ros_pose.orientation.x);
    proto_pose.mutable_rotation()->set_y(ros_pose.orientation.y);
    proto_pose.mutable_rotation()->set_z(ros_pose.orientation.z);
}


class PoseToRobotCommandNode : public rclcpp::Node {
public:
  using RobotCommandAction = spot_msgs::action::RobotCommand;
  using GoalHandleRobotCommand = rclcpp_action::ClientGoalHandle<RobotCommandAction>;

  PoseToRobotCommandNode() : Node("pose_to_robot_command_translator") {
    this->declare_parameter<std::string>("input_pose_topic", "body_pose_target");
    this->declare_parameter<std::string>("robot_command_action_name", "robot_command");
    this->declare_parameter<std::string>("spot_name", "");
    this->declare_parameter<double>("command_duration_secs", 0.25);

    std::string input_pose_topic = this->get_parameter("input_pose_topic").as_string();
    std::string robot_command_action_name = this->get_parameter("robot_command_action_name").as_string();
    std::string spot_name = this->get_parameter("spot_name").as_string();
    command_duration_secs_ = this->get_parameter("command_duration_secs").as_double();

    std::string full_action_name = robot_command_action_name;
    if (!spot_name.empty()) {
      full_action_name = "/" + spot_name + "/" + robot_command_action_name;
    }

    pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        input_pose_topic, 10,
        std::bind(&PoseToRobotCommandNode::pose_callback, this, std::placeholders::_1));

    robot_command_client_ = rclcpp_action::create_client<RobotCommandAction>(
        this, full_action_name);

    RCLCPP_INFO(this->get_logger(), "'%s' started.", this->get_name());
    RCLCPP_INFO(this->get_logger(), "  Subscribing to Pose on: %s", input_pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Sending RobotCommand actions to: %s", full_action_name.c_str());
    RCLCPP_INFO(this->get_logger(), "  Command duration for pose point: %.2f s", command_duration_secs_);

    if (!robot_command_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "RobotCommand action server '%s' not available after 10s.", full_action_name.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Successfully connected to RobotCommand action server '%s'.", full_action_name.c_str());
    }
  }

private:
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!robot_command_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "RobotCommand action server not available.");
      return;
    }

    auto goal_msg = RobotCommandAction::Goal(); // spot_msgs::action::RobotCommand::Goal
                                                // goal_msg.command is of type bosdyn_api_msgs::msg::RobotCommand

    // Access the inner 'command' field (RobotCommandOneOfCommand)
    auto& one_of_command = goal_msg.command.command; // Type: bosdyn_api_msgs::msg::RobotCommandOneOfCommand
    one_of_command.command_choice = bosdyn_api_msgs::msg::RobotCommandOneOfCommand::COMMAND_SYNCHRONIZED_COMMAND_SET;
    
    // Access synchronized_command from the one_of_command
    auto& sync_cmd_req = one_of_command.synchronized_command; // Type: bosdyn_api_msgs::msg::SynchronizedCommandRequest

    sync_cmd_req.has_field |= bosdyn_api_msgs::msg::SynchronizedCommandRequest::MOBILITY_COMMAND_FIELD_SET;
    auto& mobility_cmd_req = sync_cmd_req.mobility_command; // Type: bosdyn_api_msgs::msg::MobilityCommandRequest

    // Create bosdyn.api.spot.BodyControlParams (protobuf)
    bosdyn::api::spot::BodyControlParams body_control_params_proto;
    bosdyn::api::SE3TrajectoryPoint* traj_point_proto = body_control_params_proto.mutable_base_offset_rt_footprint()->add_points();
    
    convertRosPoseToProtoSE3Pose(*msg, *traj_point_proto->mutable_pose());
    traj_point_proto->mutable_time_since_reference()->set_seconds(static_cast<int64_t>(std::floor(command_duration_secs_)));
    traj_point_proto->mutable_time_since_reference()->set_nanos(static_cast<int32_t>((command_duration_secs_ - std::floor(command_duration_secs_)) * 1e9));

    // Create bosdyn.api.spot.MobilityParams (protobuf)
    bosdyn::api::spot::MobilityParams mobility_params_proto;
    mobility_params_proto.mutable_body_control()->CopyFrom(body_control_params_proto);
    // Locomotion hint and stair hint could be set here if needed, e.g.
    // mobility_params_proto.set_locomotion_hint(bosdyn::api::spot::HINT_AUTO);

    // Serialize MobilityParams into google.protobuf.Any (protobuf)
    google::protobuf::Any any_proto_msg; 
    any_proto_msg.PackFrom(mobility_params_proto);

    // Set the params field in the ROS MobilityCommandRequest message
    mobility_cmd_req.has_field |= bosdyn_api_msgs::msg::MobilityCommandRequest::PARAMS_FIELD_SET;
    mobility_cmd_req.params.type_url = any_proto_msg.type_url();
    const std::string& serialized_value = any_proto_msg.value();
    mobility_cmd_req.params.value.assign(serialized_value.begin(), serialized_value.end());

    RCLCPP_DEBUG(this->get_logger(), "Sending body pose command via RobotCommand (AnyProto): Pos Z=%.3f, Orient (W=%.3f, X=%.3f, Y=%.3f, Z=%.3f) duration %.2fs",
                  msg->position.z, msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z, command_duration_secs_);

    auto send_goal_options = rclcpp_action::Client<RobotCommandAction>::SendGoalOptions();
    robot_command_client_->async_send_goal(goal_msg, send_goal_options);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp_action::Client<RobotCommandAction>::SharedPtr robot_command_client_;
  double command_duration_secs_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseToRobotCommandNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
