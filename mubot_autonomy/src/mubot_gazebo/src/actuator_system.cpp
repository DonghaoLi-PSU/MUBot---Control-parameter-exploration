// Gazebo (gz sim) system driving μBot's joints. Replaces legacy
// actuator_plugin.cc: no CSV polling, commands arrive on a gz topic,
// any number of active joints, bias term, watchdog and e-stop.

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "mubot_gazebo/gait_cmd_packing.hpp"
#include "mubot_gazebo/motor_model.hpp"

namespace mubot_gazebo
{

class ActuatorSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    auto elem = std::const_pointer_cast<sdf::Element>(sdf);
    for (auto e = elem->FindElement("joint_name"); e; e = e->GetNextElement("joint_name")) {
      active_.push_back(AddJoint(model, ecm, e->Get<std::string>()));
    }
    passive_ = AddJoint(model, ecm, sdf->Get<std::string>("passive_joint", "joint_p0").first);
    timeout_ = sdf->Get<double>("watchdog_timeout", 0.2).first;
    const auto cmd_topic = sdf->Get<std::string>("command_topic", "/model/mubot/gait_cmd").first;
    const auto estop_topic = sdf->Get<std::string>("estop_topic", "/model/mubot/estop").first;
    const auto state_topic =
      sdf->Get<std::string>("state_topic", "/model/mubot/actuator_state").first;

    node_.Subscribe(cmd_topic, &ActuatorSystem::OnCommand, this);
    node_.Subscribe(estop_topic, &ActuatorSystem::OnEstop, this);
    state_pub_ = node_.Advertise<gz::msgs::Double_V>(state_topic);
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {return;}
    const double t = std::chrono::duration<double>(info.simTime).count();
    const double dt = std::chrono::duration<double>(info.dt).count();

    std::lock_guard<std::mutex> lock(mutex_);
    if (fresh_) {
      last_cmd_time_ = t;
      fresh_ = false;
      if (!active_cmd_ || !pending_->switch_at_phase) {
        active_cmd_ = pending_;
        pending_.reset();
      }
    }

    // Integrate the gait phase so frequency changes don't cause jumps.
    const double f = active_cmd_ ? active_cmd_->frequency : 0.0;
    const double prev_phase = phase_;
    phase_ = std::fmod(phase_ + f * dt, 1.0);
    if (pending_ && phase_ < prev_phase) {  // phase wrapped: apply the queued switch
      active_cmd_ = pending_;
      pending_.reset();
    }

    const bool tripped = active_cmd_ && (t - last_cmd_time_ > timeout_);
    const bool drive = active_cmd_ && !tripped && !estop_;
    const double K = active_cmd_ ? active_cmd_->stiffness : 0.0;
    const double tail_ratio = active_cmd_ ? active_cmd_->tail_ratio : 5.0;

    ActuatorStateData state;
    state.watchdog_tripped = tripped;
    state.estop = estop_;
    for (std::size_t i = 0; i < active_.size(); ++i) {
      const double q = Position(active_[i], ecm);
      const double qd = Velocity(active_[i], ecm);
      double v = 0.0;
      bool clipped = false;
      if (drive && i < active_cmd_->amplitude.size()) {
        const double raw = active_cmd_->amplitude[i] *
          std::sin(2.0 * M_PI * (phase_ + active_cmd_->phase[i])) + active_cmd_->bias;
        v = clip_voltage(raw, motor_, &clipped);
      }
      const double tau = motor_torque(v, qd, motor_) + spring_torque(K, q, motor_);
      active_[i].SetForce(ecm, {tau});
      state.voltage.push_back(v);
      state.clipped.push_back(clipped);
      state.power += electrical_power(v, qd, motor_);
    }
    passive_.SetForce(ecm, {spring_torque(tail_ratio * K, Position(passive_, ecm), motor_)});

    if (++publish_counter_ % 80 == 0) {  // 4 kHz / 80 = 50 Hz
      gz::msgs::Double_V msg;
      for (double d : pack(state)) {msg.add_data(d);}
      state_pub_.Publish(msg);
    }
  }

private:
  static gz::sim::Joint AddJoint(
    gz::sim::Model & model, gz::sim::EntityComponentManager & ecm, const std::string & name)
  {
    gz::sim::Joint j(model.JointByName(ecm, name));
    j.EnablePositionCheck(ecm, true);
    j.EnableVelocityCheck(ecm, true);
    return j;
  }

  static double Position(const gz::sim::Joint & j, const gz::sim::EntityComponentManager & ecm)
  {
    const auto p = j.Position(ecm);
    return (p && !p->empty()) ? p->at(0) : 0.0;
  }

  static double Velocity(const gz::sim::Joint & j, const gz::sim::EntityComponentManager & ecm)
  {
    const auto v = j.Velocity(ecm);
    return (v && !v->empty()) ? v->at(0) : 0.0;
  }

  void OnCommand(const gz::msgs::Double_V & msg)
  {
    std::vector<double> d(msg.data().begin(), msg.data().end());
    auto cmd = unpack_gait(d);
    if (!cmd) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = cmd;
    fresh_ = true;
  }

  void OnEstop(const gz::msgs::Boolean & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    estop_ = msg.data();
  }

  std::vector<gz::sim::Joint> active_;
  gz::sim::Joint passive_{gz::sim::kNullEntity};
  MotorParams motor_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher state_pub_;

  std::mutex mutex_;
  std::optional<GaitCommand> active_cmd_, pending_;
  bool fresh_ = false;
  bool estop_ = false;
  double last_cmd_time_ = 0.0;
  double timeout_ = 0.2;
  double phase_ = 0.0;
  unsigned publish_counter_ = 0;
};

}  // namespace mubot_gazebo

GZ_ADD_PLUGIN(
  mubot_gazebo::ActuatorSystem, gz::sim::System,
  mubot_gazebo::ActuatorSystem::ISystemConfigure,
  mubot_gazebo::ActuatorSystem::ISystemPreUpdate)
