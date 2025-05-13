#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "yaml-cpp/yaml.h" // Ensure this is included
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <memory>

// Service headers
#include "std_srvs/srv/trigger.hpp"
#include "spot_msgs/srv/dock.hpp"

// Structure to hold a single button-to-service mapping configuration
struct ButtonServiceMapping {
    int button_id;
    std::string service_name;
    std::string service_type_name;
    YAML::Node request_args_yaml;
    rclcpp::ClientBase::SharedPtr generic_client;
};

class JoyServiceCallerNode : public rclcpp::Node {
public:
    JoyServiceCallerNode() : Node("joy_service_caller") {
        this->declare_parameter<std::string>("mappings_file_path", "default_joy_mappings.yaml"); // Provide a default
        std::string mappings_path = this->get_parameter("mappings_file_path").as_string();

        load_service_mappings(mappings_path);

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyServiceCallerNode::joy_message_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "'%s' started. Loaded %zu service mappings.",
                    this->get_name(), button_service_mappings_.size());
        for (const auto& mapping : button_service_mappings_) {
            RCLCPP_INFO(this->get_logger(), "  Mapping: Button %d -> %s (%s)",
                        mapping.button_id, mapping.service_name.c_str(), mapping.service_type_name.c_str());
        }
        if (button_service_mappings_.empty() && !mappings_path.empty()) {
             RCLCPP_WARN(this->get_logger(), "No service mappings were loaded from '%s'. Check file content and path.", mappings_path.c_str());
        }
    }

private:
    void load_service_mappings(const std::string& file_path) {
        if (file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Mappings file path parameter is empty.");
            return;
        }

        try {
            YAML::Node config_root = YAML::LoadFile(file_path);
            std::string node_name_str = this->get_name(); // "joy_service_caller"
            YAML::Node service_mappings_list_node;

            // Expected structure: config_root -> "joy_service_caller" -> "ros__parameters" -> "service_mappings"
            if (config_root[node_name_str] &&
                config_root[node_name_str]["ros__parameters"] &&
                config_root[node_name_str]["ros__parameters"]["service_mappings"]) {
                service_mappings_list_node = config_root[node_name_str]["ros__parameters"]["service_mappings"];
            } else {
                RCLCPP_ERROR(this->get_logger(), "Could not find 'service_mappings' at the expected path '%s.ros__parameters.service_mappings' in file: %s",
                             node_name_str.c_str(), file_path.c_str());
                // Log details for debugging
                if (!config_root[node_name_str]) {
                    RCLCPP_INFO(this->get_logger(), "Debug: Top-level key '%s' not found in YAML.", node_name_str.c_str());
                } else if (!config_root[node_name_str]["ros__parameters"]) {
                    RCLCPP_INFO(this->get_logger(), "Debug: Key 'ros__parameters' not found under '%s' in YAML.", node_name_str.c_str());
                } else if (!config_root[node_name_str]["ros__parameters"]["service_mappings"]) {
                    RCLCPP_INFO(this->get_logger(), "Debug: Key 'service_mappings' not found under '%s.ros__parameters' in YAML.", node_name_str.c_str());
                }
                return; // Exit if not found at the primary expected path
            }

            if (service_mappings_list_node.IsSequence()) {
                for (const auto& item : service_mappings_list_node) {
                    ButtonServiceMapping mapping;
                    // Ensure the YAML key "button_index" is used here as per your YAML file
                    if (!item["button_index"]) {
                        RCLCPP_WARN(this->get_logger(), "Skipping mapping: 'button_index' not found in an item.");
                        continue;
                    }
                    mapping.button_id = item["button_index"].as<int>();

                    if (!item["service_name"]) {
                        RCLCPP_WARN(this->get_logger(), "Skipping mapping for button %d: 'service_name' not found.", mapping.button_id);
                        continue;
                    }
                    mapping.service_name = item["service_name"].as<std::string>();

                    if (!item["service_type"]) {
                        RCLCPP_WARN(this->get_logger(), "Skipping mapping for button %d: 'service_type' not found.", mapping.button_id);
                        continue;
                    }
                    mapping.service_type_name = item["service_type"].as<std::string>();

                    if (item["request_payload"]) {
                        mapping.request_args_yaml = item["request_payload"];
                    }
                    button_service_mappings_.push_back(mapping);
                }
            } else {
                 RCLCPP_ERROR(this->get_logger(), "'service_mappings' was found but is not a YAML sequence in file: %s", file_path.c_str());
            }

        } catch (const YAML::BadFile& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mappings file (BadFile): '%s'. Error: %s", file_path.c_str(), e.what());
        } catch (const YAML::ParserException& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse mappings file (ParserException): '%s'. Error: %s", file_path.c_str(), e.what());
        } catch (const YAML::Exception& e) { // Catch other YAML exceptions
            RCLCPP_ERROR(this->get_logger(), "YAML-related error processing file '%s': %s", file_path.c_str(), e.what());
        } catch (const std::exception& e) { // Catch standard exceptions
            RCLCPP_ERROR(this->get_logger(), "General error loading service mappings from '%s': %s", file_path.c_str(), e.what());
        }
    }

    // ... (rest of your JoyServiceCallerNode class: joy_message_callback, process_service_request, etc.)
    void joy_message_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        if (previous_button_states_.empty() && !joy_msg->buttons.empty()) {
            previous_button_states_.resize(joy_msg->buttons.size(), 0);
        }
        if (joy_msg->buttons.size() != previous_button_states_.size()){
             previous_button_states_.resize(joy_msg->buttons.size(), 0); // Resize if joy buttons array changes
        }


        for (size_t i = 0; i < button_service_mappings_.size(); ++i) {
            ButtonServiceMapping& current_mapping = button_service_mappings_[i]; // Use reference for client creation
            if (current_mapping.button_id < 0 || static_cast<size_t>(current_mapping.button_id) >= joy_msg->buttons.size()) {
                continue;
            }

            int current_button_state = joy_msg->buttons[current_mapping.button_id];
            int previous_state = previous_button_states_[current_mapping.button_id];

            if (current_button_state == 1 && previous_state == 0) { // Rising edge detection
                RCLCPP_INFO(this->get_logger(), "Button %d pressed. Processing service call for '%s'.",
                            current_mapping.button_id, current_mapping.service_name.c_str());
                process_service_request(current_mapping);
            }
        }
        previous_button_states_ = joy_msg->buttons;
    }

    void process_service_request(ButtonServiceMapping& mapping_config) {
        if (mapping_config.service_type_name == "std_srvs/srv/Trigger") {
            invoke_trigger_service(mapping_config);
        } else if (mapping_config.service_type_name == "spot_msgs/srv/Dock") {
            invoke_dock_service(mapping_config);
        } else {
            RCLCPP_WARN(this->get_logger(), "Service type '%s' for service '%s' is not supported.",
                        mapping_config.service_type_name.c_str(), mapping_config.service_name.c_str());
        }
    }

    template<typename ServiceT>
    typename rclcpp::Client<ServiceT>::SharedPtr get_or_create_client(ButtonServiceMapping& mapping_config) {
        if (!mapping_config.generic_client) {
            mapping_config.generic_client = this->create_client<ServiceT>(mapping_config.service_name);
            RCLCPP_INFO(this->get_logger(), "Created new client for service '%s' (%s).",
                        mapping_config.service_name.c_str(), mapping_config.service_type_name.c_str());
        }
        return std::static_pointer_cast<rclcpp::Client<ServiceT>>(mapping_config.generic_client);
    }

    void invoke_trigger_service(ButtonServiceMapping& mapping_config) {
        auto trigger_client = get_or_create_client<std_srvs::srv::Trigger>(mapping_config);

        if (!trigger_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service '%s' (Trigger) is not available.", mapping_config.service_name.c_str());
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        // Trigger request is empty.

        trigger_client->async_send_request(request,
            [this, service_name = mapping_config.service_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_response) {
                try {
                    auto response = future_response.get();
                    RCLCPP_INFO(this->get_logger(), "Service '%s' (Trigger) call result: %s - Message: '%s'",
                                service_name.c_str(), response->success ? "SUCCESS" : "FAILURE", response->message.c_str());
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception during service '%s' (Trigger) call: %s", service_name.c_str(), e.what());
                }
            });
    }

    void invoke_dock_service(ButtonServiceMapping& mapping_config) {
        auto dock_client = get_or_create_client<spot_msgs::srv::Dock>(mapping_config);

        if (!dock_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service '%s' (Dock) is not available.", mapping_config.service_name.c_str());
            return;
        }

        auto request = std::make_shared<spot_msgs::srv::Dock::Request>();
        if (mapping_config.request_args_yaml && mapping_config.request_args_yaml["dock_id"]) { // Check if request_args_yaml is valid
            try {
                request->dock_id = mapping_config.request_args_yaml["dock_id"].as<uint32_t>();
                RCLCPP_DEBUG(this->get_logger(), "Dock service '%s' request dock_id: %u", mapping_config.service_name.c_str(), request->dock_id);
            } catch (const YAML::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse 'dock_id' for service '%s': %s. Using default/empty request.",
                             mapping_config.service_name.c_str(), e.what());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "'dock_id' not found in 'request_payload' for service '%s', or payload is missing. Consult YAML configuration.", mapping_config.service_name.c_str());
        }

        dock_client->async_send_request(request,
            [this, service_name = mapping_config.service_name](rclcpp::Client<spot_msgs::srv::Dock>::SharedFuture future_response) {
                try {
                    auto response = future_response.get();
                    RCLCPP_INFO(this->get_logger(), "Service '%s' (Dock) call result: %s - Message: '%s'",
                                service_name.c_str(), response->success ? "SUCCESS" : "FAILURE", response->message.c_str());
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception during service '%s' (Dock) call: %s", service_name.c_str(), e.what());
                }
            });
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    std::vector<ButtonServiceMapping> button_service_mappings_;
    std::vector<int> previous_button_states_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto joy_service_caller_node = std::make_shared<JoyServiceCallerNode>();
    rclcpp::spin(joy_service_caller_node);
    rclcpp::shutdown();
    return 0;
}
