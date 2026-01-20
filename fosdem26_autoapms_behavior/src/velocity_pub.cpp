// Copyright 2026 Robin Müller
//
// Licensed under the MIT License

#include "auto_apms_behavior_tree/node.hpp"
#include "fosdem26_autoapms_interfaces/msg/robot_velocity.hpp"

namespace fosdem26_autoapms_behavior
{

/**
 * @brief Behavior tree node that publishes velocity commands to the robot.
 *
 * This node publishes a RobotVelocity message to control the robot hand's movement.
 * The velocity value is read from an input port and published to the configured topic.
 */
class VelocityPub
: public auto_apms_behavior_tree::core::RosPublisherNode<fosdem26_autoapms_interfaces::msg::RobotVelocity>
{
public:
  explicit VelocityPub(const std::string & instance_name, const Config & config, Context context)
  : RosPublisherNode(instance_name, config, context, rclcpp::SensorDataQoS()) {};

  /**
   * @brief Define the input/output ports for this node.
   * @return List of ports including the velocity input.
   */
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("velocity", 0.0, "Velocity command (-1.0 to 1.0)")};
  }

  /**
   * @brief Set the velocity message content before publishing.
   * @param msg The message to populate with velocity data.
   * @return true if the message was successfully set, false to abort publishing.
   */
  bool setMessage(fosdem26_autoapms_interfaces::msg::RobotVelocity & msg) override
  {
    double velocity;
    if (!getInput("velocity", velocity)) {
      RCLCPP_ERROR(logger_, "Failed to get 'velocity' from input port");
      return false;
    }
    msg.velocity = velocity;
    return true;
  }
};

}  // namespace fosdem26_autoapms_behavior

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(fosdem26_autoapms_behavior::VelocityPub)
