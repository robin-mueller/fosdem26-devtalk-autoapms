// Copyright 2026 Robin Müller
//
// Licensed under the MIT License

#include "auto_apms_behavior_tree/node.hpp"
#include "fosdem26_autoapms_interfaces/msg/robot_position.hpp"

namespace fosdem26_autoapms_behavior
{

/**
 * @brief Behavior tree node that checks if the robot position is within a specified range.
 *
 * This node subscribes to the robot position topic and returns SUCCESS if the position
 * is within the range [low, high], otherwise returns FAILURE.
 */
class PositionInRange
: public auto_apms_behavior_tree::core::RosSubscriberNode<fosdem26_autoapms_interfaces::msg::RobotPosition>
{
public:
  explicit PositionInRange(const std::string & instance_name, const Config & config, Context context)
  : RosSubscriberNode(instance_name, config, context, rclcpp::SensorDataQoS()) {};

  /**
   * @brief Define the input/output ports for this node.
   * @return List of ports including the range bounds.
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("low", -1.0, "Lower bound of the position range"),
      BT::InputPort<double>("high", 1.0, "Upper bound of the position range")};
  }

  /**
   * @brief Process the received position message.
   * @param msg The received RobotPosition message.
   * @return SUCCESS if position is in range, FAILURE otherwise.
   */
  BT::NodeStatus onMessageReceived(const fosdem26_autoapms_interfaces::msg::RobotPosition & msg) override
  {
    double low, high;
    if (!getInput("low", low)) {
      RCLCPP_ERROR(logger_, "Failed to get 'low' from input port");
      return BT::NodeStatus::FAILURE;
    }
    if (!getInput("high", high)) {
      RCLCPP_ERROR(logger_, "Failed to get 'high' from input port");
      return BT::NodeStatus::FAILURE;
    }

    if (msg.position >= low && msg.position <= high) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace fosdem26_autoapms_behavior

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(fosdem26_autoapms_behavior::PositionInRange)
