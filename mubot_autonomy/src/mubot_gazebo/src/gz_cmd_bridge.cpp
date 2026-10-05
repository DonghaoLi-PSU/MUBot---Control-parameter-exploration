// Bridges the custom GaitCmd / ActuatorState messages between ROS 2 and
// gz-transport (gz.msgs.Double_V). Standard types go through ros_gz_bridge.

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

#include "mubot_gazebo/gait_cmd_packing.hpp"
#include "mubot_interfaces/msg/actuator_state.hpp"
#include "mubot_interfaces/msg/gait_cmd.hpp"

namespace mubot_gazebo
{

class GzCmdBridge : public rclcpp::Node
{
public:
  GzCmdBridge()
  : Node("mubot_gz_cmd_bridge")
  {
    const auto gz_cmd = declare_parameter("gz_command_topic", "/model/mubot/gait_cmd");
    const auto gz_state = declare_parameter("gz_state_topic", "/model/mubot/actuator_state");

    gz_pub_ = gz_node_.Advertise<gz::msgs::Double_V>(gz_cmd);
    state_pub_ = create_publisher<mubot_interfaces::msg::ActuatorState>(
      "/mubot/actuator_state", rclcpp::QoS(10));

    rclcpp::QoS cmd_qos(1);
    cmd_qos.reliable().deadline(std::chrono::milliseconds(100));
    cmd_sub_ = create_subscription<mubot_interfaces::msg::GaitCmd>(
      "/mubot/gait_cmd", cmd_qos,
      [this](const mubot_interfaces::msg::GaitCmd & m) {
        GaitCommand c{m.amplitude, m.phase, m.frequency, m.bias, m.stiffness, m.tail_ratio,
          m.switch_at_phase};
        gz::msgs::Double_V out;
        for (double d : pack(c)) {out.add_data(d);}
        gz_pub_.Publish(out);
      });

    gz_node_.Subscribe(gz_state, &GzCmdBridge::OnState, this);
  }

private:
  void OnState(const gz::msgs::Double_V & msg)
  {
    std::vector<double> d(msg.data().begin(), msg.data().end());
    const auto s = unpack_state(d);
    if (!s) {return;}
    mubot_interfaces::msg::ActuatorState out;
    out.header.stamp = now();
    out.voltage = s->voltage;
    out.clipped.assign(s->clipped.begin(), s->clipped.end());
    out.power = s->power;
    out.watchdog_tripped = s->watchdog_tripped;
    out.estop = s->estop;
    state_pub_->publish(out);
  }

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher gz_pub_;
  rclcpp::Publisher<mubot_interfaces::msg::ActuatorState>::SharedPtr state_pub_;
  rclcpp::Subscription<mubot_interfaces::msg::GaitCmd>::SharedPtr cmd_sub_;
};

}  // namespace mubot_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mubot_gazebo::GzCmdBridge>());
  rclcpp::shutdown();
  return 0;
}
