// Gazebo (gz sim) system applying the μBot hydrodynamic model to every segment.
// Replaces legacy Hydro_plugin.cc. The force math lives in hydro_model.hpp.
//
// TODO(port): verify against the legacy plugin with tools/compare_baseline.py.
// Points to check: whether WorldLinearAcceleration refers to the link origin
// or the centre of mass (legacy used the CoG acceleration), and whether
// AddWorldForce applies at the centre of mass (legacy AddLinkForce did).

#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

#include "mubot_gazebo/hydro_model.hpp"

namespace mubot_gazebo
{

class HydroSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    LoadParams(sdf->Get<std::string>("body_params"), sdf->Get<std::string>("model_params"));
    calibration_time_ = sdf->Get<double>("calibration_time", 0.1).first;

    // Segment order: head, body_1..n, tail, tip.
    std::vector<std::string> names{"segment_head"};
    for (int i = 1; ; ++i) {
      const auto name = "segment_body_" + std::to_string(i);
      if (model_.LinkByName(ecm, name) == gz::sim::kNullEntity) {break;}
      names.push_back(name);
    }
    names.push_back("segment_tail");
    names.push_back("segment_tip");
    for (const auto & n : names) {
      gz::sim::Link link(model_.LinkByName(ecm, n));
      link.EnableVelocityChecks(ecm, true);
      link.EnableAccelerationChecks(ecm, true);
      links_.push_back(link);
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {return;}
    const double t = std::chrono::duration<double>(info.simTime).count();
    const int s_num = static_cast<int>(links_.size());

    for (int i = 0; i < s_num; ++i) {
      auto & link = links_[i];
      if (t <= calibration_time_) {
        // Legacy "calibration": hold the body still while the joints settle.
        link.SetLinearVelocity(ecm, gz::math::Vector3d::Zero);
        link.SetAngularVelocity(ecm, gz::math::Vector3d::Zero);
        continue;
      }
      const auto pose = link.WorldPose(ecm);
      const auto v = link.WorldLinearVelocity(ecm);
      const auto w = link.WorldAngularVelocity(ecm);
      const auto a = link.WorldLinearAcceleration(ecm);
      const auto alpha = link.WorldAngularAcceleration(ecm);
      if (!pose || !v || !w || !a || !alpha) {continue;}

      SegmentKinematics k{pose->Rot().Yaw(), v->X(), v->Y(), a->X(), a->Y(), w->Z(), alpha->Z()};
      const SegmentWrench f = compute_wrench(params_, segment_type(i, s_num), k);

      // Link-frame force → world frame.
      const double c = std::cos(k.yaw), s = std::sin(k.yaw);
      const gz::math::Vector3d force_world(f.fx * c - f.fy * s, f.fx * s + f.fy * c, 0.0);
      link.AddWorldForce(ecm, force_world);
      link.AddWorldWrench(ecm, gz::math::Vector3d::Zero, gz::math::Vector3d(0, 0, f.tz));
    }
  }

private:
  void LoadParams(const std::string & body_file, const std::string & model_file)
  {
    const YAML::Node b = YAML::LoadFile(body_file);
    const YAML::Node m = YAML::LoadFile(model_file);
    params_.Ca = m["Ca"].as<double>();
    params_.Cp = m["Cp"].as<double>();
    params_.rho = m["rho"].as<double>();
    params_.M_total_ratio = m["M_total_ratio"].as<double>();
    params_.c_d = b["c_d"].as<double>();
    params_.c_f = b["c_f"].as<double>();
    params_.M_0 = b["M_0"].as<double>();
    params_.M_l = b["M_l"].as<double>();
    params_.h_0 = b["h_0"].as<double>();
    for (int j = 0; j < 4; ++j) {
      params_.seg[j] = SegmentHydro{
        b["M_total"][j].as<double>(), b["M_S"][j].as<double>(), b["cog"][j].as<double>(),
        b["lt"][j].as<double>(), b["IH0"][j].as<double>(), b["IH1"][j].as<double>(),
        b["IH2"][j].as<double>(), b["IH3"][j].as<double>(), b["IP"][j].as<double>()};
    }
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::vector<gz::sim::Link> links_;
  HydroParams params_;
  double calibration_time_ = 0.1;
};

}  // namespace mubot_gazebo

GZ_ADD_PLUGIN(
  mubot_gazebo::HydroSystem, gz::sim::System,
  mubot_gazebo::HydroSystem::ISystemConfigure,
  mubot_gazebo::HydroSystem::ISystemPreUpdate)
