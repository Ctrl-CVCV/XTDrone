#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

// car3 X-type mecanum inverse kinematics.
// Geometry (measured from car3.urdf / STL):
//   wheel radius r = 0.0489 m
//   L = 0.0973 m (x half-distance), W = 0.1058 m (y half-distance)
// ROS convention: +x forward, +y left, +z up.
//
// Theoretical starting formulas (signs validated by motion tests):
//   w_fl = (vx - vy - (L+W)*wz) / r
//   w_fr = (vx + vy + (L+W)*wz) / r
//   w_rl = (vx + vy - (L+W)*wz) / r
//   w_rr = (vx - vy + (L+W)*wz) / r
// Each wheel has a sign parameter to correct joint-axis direction
// mismatches found during motion tests (defaults 1.0).

class MecanumController
{
public:
  MecanumController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param("wheel_radius", r_, 0.0489);
    pnh.param("wheel_base_x_half", L_, 0.0973);
    pnh.param("wheel_base_y_half", W_, 0.1058);
    pnh.param("sign_fl", s_[0], 1.0);
    pnh.param("sign_fr", s_[1], 1.0);
    pnh.param("sign_rl", s_[2], 1.0);
    pnh.param("sign_rr", s_[3], 1.0);

    pubs_[0] = nh.advertise<std_msgs::Float64>("/wheel_lf_velocity_controller/command", 10);
    pubs_[1] = nh.advertise<std_msgs::Float64>("/wheel_rf_velocity_controller/command", 10);
    pubs_[2] = nh.advertise<std_msgs::Float64>("/wheel_lb_velocity_controller/command", 10);
    pubs_[3] = nh.advertise<std_msgs::Float64>("/wheel_rb_velocity_controller/command", 10);

    sub_ = nh.subscribe("/cmd_vel", 10, &MecanumController::cmdVelCallback, this);

    ROS_INFO("mecanum_controller: r=%.4f L=%.4f W=%.4f signs=[%.0f %.0f %.0f %.0f]",
             r_, L_, W_, s_[0], s_[1], s_[2], s_[3]);
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double wz = msg->angular.z;

    // X-type mecanum: FL, FR, RL, RR
    double w[4];
    w[0] = s_[0] * (vx - vy - (L_ + W_) * wz) / r_;
    w[1] = s_[1] * (vx + vy + (L_ + W_) * wz) / r_;
    w[2] = s_[2] * (vx + vy - (L_ + W_) * wz) / r_;
    w[3] = s_[3] * (vx - vy + (L_ + W_) * wz) / r_;

    for (int i = 0; i < 4; ++i)
    {
      std_msgs::Float64 cmd;
      cmd.data = w[i];
      pubs_[i].publish(cmd);
    }
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pubs_[4];
  double r_, L_, W_;
  double s_[4];
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mecanum_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  MecanumController controller(nh, pnh);
  ros::spin();
  return 0;
}
