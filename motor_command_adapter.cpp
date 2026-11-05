// motor_command_adapter.cpp
// Original working source supplied by Erhan; explanatory comments added.
// Executable statements, topic names, queue depths, and validation are unchanged.
//
// PURPOSE IN THE CO-SIMULATION
// Simulink publishes four motor-speed references using Float64MultiArray.
// Gazebo's motor-command topic expects an Actuators message after translation
// through ros_gz_bridge. This node copies the speed array between these types:
//
// Input ROS topic:  /quadrotor/matlab_motor_commands
// Input ROS type:   std_msgs/msg/Float64MultiArray
// Output ROS topic: /quadrotor/command/motor_speed
// Output ROS type:  actuator_msgs/msg/Actuators
// Output field:     velocity
//
// Units: rad/s. Values are motor-speed magnitudes, not RPM or thrust in newtons.
// Array order must match the SDF motorNumber values:
//   data[0] = front, data[1] = right, data[2] = back, data[3] = left.
// The adapter preserves this order; it does not discover rotor positions.
// Rotor rotation directions are specified separately in the SDF motor plugins.
//
// Feed the references BEFORE Simulink's motor dynamics to this node. Gazebo
// applies its own first-order motor response; sending already-filtered speeds
// would apply a second motor response in that branch.
//
// This is message conversion. There is no state feedback, ADRC, or autopilot
// here. The separate ros_gz_bridge process handles ROS/Gazebo Transport types.

// std::bind and its placeholder connect a member function to the subscription.
#include <functional>
// std::shared_ptr and std::make_shared manage node/message ownership.
#include <memory>

// ROS 2 C++ node, publisher, subscription, logging, and execution APIs.
#include "rclcpp/rclcpp.hpp"
// Simulink's incoming message: the speeds are stored in msg->data.
#include "std_msgs/msg/float64_multi_array.hpp"
// Outgoing motor message: the speeds are stored in output.velocity.
#include "actuator_msgs/msg/actuators.hpp"

// Inheriting from Node gives this class access to the ROS communication APIs.
class MotorCommandAdapter : public rclcpp::Node
{
public:
    // This is the ROS node name, distinct from the package and topic names.
    MotorCommandAdapter() : Node("motor_command_adapter")
    {
        // Create the outgoing publisher. The integer 10 is a history depth;
        // it is NOT a publishing frequency, number of motors, or sample period.
        publisher_ =
            create_publisher<actuator_msgs::msg::Actuators>(
                "/quadrotor/command/motor_speed", 10);

        // Register a subscription. ROS invokes commandCallback for each
        // incoming message processed by the executor. This node has no timer:
        // its outgoing updates are driven by incoming MATLAB messages.
        subscriber_ =
            create_subscription<std_msgs::msg::Float64MultiArray>(
                "/quadrotor/matlab_motor_commands", 10,
                std::bind(
                    &MotorCommandAdapter::commandCallback,
                    this, std::placeholders::_1));
    }

private:
    // 'msg' is a shared pointer to the received Float64MultiArray message.
    void commandCallback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Require exactly one speed reference per motor. Reject an incorrectly
        // sized message rather than sending a partial or extra motor array.
        // No new command is published by this callback when the size is wrong.
        if (msg->data.size() != 4) {
            RCLCPP_WARN(get_logger(), "Expected four motor speeds.");
            return;
        }

        // Construct a new Actuators message and copy the four values directly.
        // No scaling, rotor reordering, saturation, or additional motor lag is
        // introduced. Float64MultiArray.layout is unused by this adapter.
        actuator_msgs::msg::Actuators output;
        output.velocity = msg->data;
        publisher_->publish(output);
        // The header and other Actuators fields retain their default values.
        // This code does not create a simulation timestamp. For state analysis,
        // use the stamp from the RECEIVED Gazebo odometry message instead.
        // The original implementation checks array length only. Commands must
        // be prepared upstream with the intended finite, nonnegative magnitudes;
        // this node has no range check, timeout, or automatic stop-on-disconnect.
    }

    // Keep these shared pointers alive for as long as the node exists.
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr
        publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        subscriber_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS and process its command-line/remapping arguments.
    // ROS_DOMAIN_ID must be set consistently in the environments that launch
    // MATLAB, this adapter, and the ROS/Gazebo bridge (our experiments use 0).
    rclcpp::init(argc, argv);
    // Process callbacks until interrupted, typically by Ctrl+C in this terminal.
    rclcpp::spin(std::make_shared<MotorCommandAdapter>());
    // Release the ROS context after the executor stops.
    rclcpp::shutdown();
    return 0;
}
