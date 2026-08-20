#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <vector>
#include <string>

// car3 X-type mecanum roller-grip model plugin.
//
// ODE surface friction cannot model mecanum rollers: fdir1 is body-fixed
// to the spinning wheel collision, so the grip direction rotates with the
// wheel and its anisotropy time-averages out. This plugin implements the
// physically correct roller grip instead: the grip direction of the roller
// currently in contact depends only on the BASE pose (constant in the wheel
// frame), not on the wheel spin angle.
//
// Per wheel: contact slip  s = v_c + w_wheel x r_contact
//   grip direction u in base frame: LF/RB (x-y)/sqrt2, RF/LB (x+y)/sqrt2
//   grip force F = -clamp(c * (s.u), mu*N) * u   applied at the contact point
// Wheel collision surfaces must have mu=mu2=0 so ODE adds no other in-plane
// force; only the normal support remains.

namespace gazebo {

class Car3MecanumGrip : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    r_ = 0.0489;
    muN_ = 52.0;
    c_ = 2000.0;
    linkPrefix_ = "";

    if (sdf->HasElement("wheel_radius")) r_ = sdf->Get<double>("wheel_radius");
    if (sdf->HasElement("grip_force_max")) muN_ = sdf->Get<double>("grip_force_max");
    if (sdf->HasElement("grip_gain")) c_ = sdf->Get<double>("grip_gain");
    if (sdf->HasElement("link_prefix")) linkPrefix_ = sdf->Get<std::string>("link_prefix");

    // grip direction in base frame (X-type mecanum)
    struct Wheel { std::string link; std::string joint; ignition::math::Vector3d grip; };
    std::vector<Wheel> cfg = {
      {linkPrefix_ + "wheel_lf_link", "wheel_lf_joint", { 0.7071, -0.7071, 0.0}},
      {linkPrefix_ + "wheel_rf_link", "wheel_rf_joint", { 0.7071,  0.7071, 0.0}},
      {linkPrefix_ + "wheel_lb_link", "wheel_lb_joint", { 0.7071,  0.7071, 0.0}},
      {linkPrefix_ + "wheel_rb_link", "wheel_rb_joint", { 0.7071, -0.7071, 0.0}},
    };

    for (const auto& w : cfg)
    {
      physics::LinkPtr link = model->GetLink(w.link);
      physics::JointPtr joint = model->GetJoint(w.joint);
      if (!link || !joint)
      {
        gzerr << "car3_mecanum_grip: missing link/joint " << w.link << " " << w.joint << "\n";
        return;
      }
      links_.push_back(link);
      joints_.push_back(joint);
      grip_.push_back(w.grip);
    }

    // the wheel joints' parent is the (possibly fixed-joint-lumped) base link
    baseLink_ = joints_[0]->GetParent();
    if (!baseLink_)
    {
      gzerr << "car3_mecanum_grip: wheel joint has no parent link\n";
      return;
    }

    updateConn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Car3MecanumGrip::OnUpdate, this));
    gzmsg << "car3_mecanum_grip: loaded on [" << model->GetName() << "], r=" << r_
          << " muN=" << muN_ << " gain=" << c_ << "\n";
  }

  void OnUpdate()
  {
    const auto pB = baseLink_->WorldPose().Pos();
    const auto R = baseLink_->WorldPose().Rot();
    const ignition::math::Vector3d vB = baseLink_->WorldLinearVel();
    const ignition::math::Vector3d wB = baseLink_->WorldAngularVel();
    const ignition::math::Vector3d xAxis = R.RotateVector({1.0, 0.0, 0.0});

    for (size_t i = 0; i < links_.size(); ++i)
    {
      const double omega = joints_[i]->GetVelocity(0);

      // wheel center (wheel link origin) and bottom contact point
      const ignition::math::Vector3d pW = links_[i]->WorldPose().Pos();
      const ignition::math::Vector3d cW = pW + R.RotateVector({0.0, 0.0, -r_});
      if (cW.Z() > 0.05)
        continue;  // wheel off the ground

      // velocity of the body at the wheel center + spin surface velocity
      const ignition::math::Vector3d vC = vB + wB.Cross(pW - pB);
      const ignition::math::Vector3d slip = vC - omega * r_ * xAxis;

      const ignition::math::Vector3d u = R.RotateVector(grip_[i]);
      const double su = slip.Dot(u);
      const double fmag = ignition::math::clamp(c_ * su, -muN_, muN_);
      const ignition::math::Vector3d F = -fmag * u;

      // applied to the base at the wheel center: the grip force is
      // transmitted through the axle; the moment (0,0,-r) x F is absorbed
      // by the wheel's own rotation, and the yaw torque is unchanged
      baseLink_->AddForceAtWorldPosition(F, pW);
    }
  }

private:
  physics::ModelPtr model_;
  physics::LinkPtr baseLink_;
  std::vector<physics::LinkPtr> links_;
  std::vector<physics::JointPtr> joints_;
  std::vector<ignition::math::Vector3d> grip_;
  event::ConnectionPtr updateConn_;
  double r_, muN_, c_;
  std::string linkPrefix_;
};

GZ_REGISTER_MODEL_PLUGIN(Car3MecanumGrip)
}  // namespace gazebo
