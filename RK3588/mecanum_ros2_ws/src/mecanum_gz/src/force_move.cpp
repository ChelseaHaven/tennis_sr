// SPDX-License-Identifier: Apache-2.0
// Harmonic port of the legacy body-force velocity loop. Wheel contacts remain
// in the physics engine; wheel friction is zero, as in the original project.
#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/math/Vector3.hh>
#include <sdf/Element.hh>

namespace mecanum_gz
{
class ForceMove : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPreUpdate
{
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    link = gz::sim::Link(model.LinkByName(ecm, sdf->Get<std::string>("link_name", "base_link").first));
    if (!link.Valid(ecm))
    {
      gzerr << "ForceMove: base link was not found\n";
      return;
    }
    link.EnableVelocityChecks(ecm, true);
    maxVelocity = sdf->Get<double>("v_max", 4.5).first;
    maxAcceleration = sdf->Get<double>("a_max", 3.0).first;
    gain = sdf->Get<double>("gain", 60.0).first;
    timeout = sdf->Get<double>("timeout", 0.25).first;
    // Mass is used only to cap force; the gain retains the old N/(m/s) units.
    const auto *inertial = ecm.Component<gz::sim::components::Inertial>(link.Entity());
    if (!inertial) { gzerr << "ForceMove: missing inertial data\n"; return; }
    mass = inertial->Data().MassMatrix().Mass();
    node.Subscribe(sdf->Get<std::string>("topic", "/cmd_vel").first,
                   &ForceMove::OnCommand, this);
    ready = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!ready || info.paused) return;
    const double now = std::chrono::duration<double>(info.simTime).count();
    gz::math::Vector3d desired;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (received) { lastCommand = now; received = false; }
      if (now < lastCommand) { command.Set(0, 0, 0); lastCommand = now; }
      if (now - lastCommand <= timeout) desired = command;
    }
    auto pose = link.WorldPose(ecm);
    auto velocity = link.WorldLinearVelocity(ecm);
    auto angular = link.WorldAngularVelocity(ecm);
    if (!pose || !velocity || !angular) return;
    const auto bodyVelocity = pose->Rot().RotateVectorReverse(*velocity);
    const double limit = mass * maxAcceleration;
    const gz::math::Vector3d force(
        std::clamp(gain * (desired.X() - bodyVelocity.X()), -limit, limit),
        std::clamp(gain * (desired.Y() - bodyVelocity.Y()), -limit, limit), 0);
    const gz::math::Vector3d torque(0, 0,
        std::clamp(desired.Z() - angular->Z(), -0.5, 0.5));
    link.AddWorldForce(ecm, pose->Rot().RotateVector(force));
    link.AddWorldWrench(ecm, gz::math::Vector3d::Zero, torque);
  }

 private:
  void OnCommand(const gz::msgs::Twist &msg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    command.Set(std::clamp(msg.linear().x(), -maxVelocity, maxVelocity),
                std::clamp(msg.linear().y(), -maxVelocity, maxVelocity),
                std::clamp(msg.angular().z(), -0.5, 0.5));
    received = true;
  }
  gz::sim::Link link;
  gz::transport::Node node;
  std::mutex mutex;
  gz::math::Vector3d command;
  double maxVelocity{4.5}, maxAcceleration{3.0}, gain{60}, timeout{0.25};
  double mass{3.0}, lastCommand{-1};
  bool received{false}, ready{false};
};
}
GZ_ADD_PLUGIN(mecanum_gz::ForceMove, gz::sim::System,
              mecanum_gz::ForceMove::ISystemConfigure,
              mecanum_gz::ForceMove::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(mecanum_gz::ForceMove, "mecanum_gz::ForceMove")
