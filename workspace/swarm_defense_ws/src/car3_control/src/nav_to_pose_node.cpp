#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <angles/angles.h>
#include <tf2/utils.h>
#include <cmath>
#include <string>
#include <deque>

// Three-phase nav_to_pose for the holonomic (mecanum) car3:
//   1. ALIGN      : rotate in place until heading == bearing to goal
//   2. TRANSLATE  : pure translation (vth forced to 0), obstacles avoided
//                   only by strafing (potential field over /scan); a head-on
//                   obstacle triggers a lateral escape (right first, left if
//                   stuck) instead of a safety stop
//   3. ROTATE_YAW : within goal_radius of the goal, rotate to the goal yaw
// All single-point checks use tolerance bands (arrive/restart hysteresis)
// so the robot never flip-flops between states.

class NavToPose
{
public:
  NavToPose()
    : nh_(), pnh_("~"), as_(nh_, "move_base", false), state_(IDLE)
  {
    pnh_.param("max_vel_trans", max_vel_trans_, 0.7);
    pnh_.param("max_vel_theta", max_vel_theta_, 0.8);
    pnh_.param("acc_lim_trans", acc_lim_trans_, 0.6);
    pnh_.param("acc_lim_theta", acc_lim_theta_, 2.0);
    pnh_.param("kp_rot", kp_rot_, 2.5);
    pnh_.param("kp_trans", kp_trans_, 1.2);
    pnh_.param("slow_radius", slow_radius_, 0.6);
    pnh_.param("align_tolerance", align_tolerance_, 0.04);
    pnh_.param("align_start_tolerance", align_start_tolerance_, 0.12);
    pnh_.param("goal_radius", goal_radius_, 0.18);
    pnh_.param("goal_radius_exit", goal_radius_exit_, 0.35);
    pnh_.param("goal_yaw_tolerance", goal_yaw_tolerance_, 0.05);
    pnh_.param("goal_yaw_start_tolerance", goal_yaw_start_tolerance_, 0.15);
    pnh_.param("settled_cycles", settled_cycles_, 5);
    pnh_.param("scan_topic", scan_topic_, std::string("scan"));
    pnh_.param("repulse_range", repulse_range_, 0.6);
    pnh_.param("repulse_gain", repulse_gain_, 0.8);
    pnh_.param("escape_angle", escape_angle_, 0.3927);
    pnh_.param("escape_range", escape_range_, 0.30);
    pnh_.param("escape_resume_clear", escape_resume_clear_, 0.6);
    pnh_.param("escape_speed", escape_speed_, 0.25);
    pnh_.param("escape_stuck_time", escape_stuck_time_, 3.0);
    pnh_.param("escape_max_time", escape_max_time_, 8.0);
    pnh_.param("escape_min_progress", escape_min_progress_, 0.08);
    pnh_.param("stall_time", stall_time_, 20.0);
    pnh_.param("stall_min_progress", stall_min_progress_, 0.08);
    pnh_.param("rate", rate_, 20.0);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pose_sub_ = nh_.subscribe("amcl_pose", 1, &NavToPose::poseCb, this);
    odom_sub_ = nh_.subscribe("odom", 1, &NavToPose::odomCb, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 1, &NavToPose::scanCb, this);
    goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &NavToPose::topicGoalCb, this);

    as_.registerGoalCallback(boost::bind(&NavToPose::actionGoalCb, this));
    as_.registerPreemptCallback(boost::bind(&NavToPose::actionPreemptCb, this));
    as_.start();

    ROS_INFO("nav_to_pose: ready (max_trans=%.2f max_theta=%.2f)",
             max_vel_trans_, max_vel_theta_);
  }

  void spin()
  {
    ros::Time prev = ros::Time::now();
    ros::Rate r(rate_);
    while (ros::ok())
    {
      ros::Time now = ros::Time::now();
      double dt = (now - prev).toSec();
      prev = now;
      if (dt <= 0.0 || dt > 0.5)
        dt = 1.0 / rate_;
      step(dt, now);
      ros::spinOnce();
      r.sleep();
    }
  }

private:
  enum State { IDLE, ALIGN, TRANSLATE, ROTATE_YAW, DONE };

  static double clamp(double v, double limit)
  {
    return std::max(-limit, std::min(limit, v));
  }

  static double ramp(double cur, double tgt, double acc, double dt)
  {
    if (cur < tgt)
      return std::min(tgt, cur + acc * dt);
    return std::max(tgt, cur - acc * dt);
  }

  void poseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;
    pose_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_pose_ = true;
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
  {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    scan_ = *msg;
    have_scan_ = true;
  }

  void topicGoalCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    if (as_.isActive())
      as_.setPreempted();
    startGoal(*msg);
  }

  void actionGoalCb()
  {
    move_base_msgs::MoveBaseGoal goal = *as_.acceptNewGoal();
    startGoal(goal.target_pose);
  }

  void actionPreemptCb()
  {
    stop("preempted");
    if (as_.isActive())
      as_.setPreempted();
  }

  void startGoal(const geometry_msgs::PoseStamped& pose)
  {
    goal_x_ = pose.pose.position.x;
    goal_y_ = pose.pose.position.y;
    goal_yaw_ = tf2::getYaw(pose.pose.orientation);
    have_goal_ = true;
    settled_ = 0;
    rotating_ = true;
    escaping_ = false;
    switched_left_ = false;
    escape_dir_ = -1.0;
    best_dist_ = -1.0;
    best_time_ = ros::Time::now();
    double dist = have_pose_ ? hypot(goal_x_ - pose_x_, goal_y_ - pose_y_) : 0.0;
    state_ = (!have_pose_ || dist >= 0.05) ? ALIGN : ROTATE_YAW;
    ROS_INFO("nav_to_pose: goal (%.2f, %.2f, %.1fdeg) dist=%.2fm -> %s",
             goal_x_, goal_y_, goal_yaw_ * 180.0 / M_PI, dist,
             state_ == ALIGN ? "ALIGN" : "ROTATE_YAW");
  }

  void stop(const char* why)
  {
    state_ = IDLE;
    have_goal_ = false;
    escaping_ = false;
    publishZero();
    ROS_INFO("nav_to_pose: stopped (%s)", why);
  }

  void publishZero()
  {
    geometry_msgs::Twist z;
    cmd_pub_.publish(z);
    cmd_vx_ = cmd_vy_ = cmd_vth_ = 0.0;
  }

  void step(double dt, const ros::Time& now)
  {
    if (state_ == IDLE || state_ == DONE)
      return;
    if (!have_pose_ || !have_goal_)
      return;

    double tgt_vx = 0.0, tgt_vy = 0.0, tgt_vth = 0.0;

    if (state_ == ALIGN)
    {
      double bearing = atan2(goal_y_ - pose_y_, goal_x_ - pose_x_);
      double err = angles::shortest_angular_distance(pose_yaw_, bearing);
      // tolerance band: start rotating only beyond the wide bound,
      // stop only within the narrow bound -> no flip-flop
      if (fabs(err) > align_start_tolerance_)
        rotating_ = true;
      else if (fabs(err) < align_tolerance_)
        rotating_ = false;

      if (rotating_)
      {
        settled_ = 0;
        tgt_vth = clamp(kp_rot_ * err, max_vel_theta_);
      }
      else
      {
        tgt_vth = 0.0;
        if (++settled_ >= settled_cycles_)
        {
          state_ = TRANSLATE;
          settled_ = 0;
          best_dist_ = hypot(goal_x_ - pose_x_, goal_y_ - pose_y_);
          best_time_ = now;
          ROS_INFO("nav_to_pose: aligned (err=%.2fdeg) -> TRANSLATE",
                   err * 180.0 / M_PI);
        }
      }
    }
    else if (state_ == TRANSLATE)
    {
      double dx = goal_x_ - pose_x_, dy = goal_y_ - pose_y_;
      double dist = hypot(dx, dy);

      if (dist < goal_radius_)
      {
        state_ = ROTATE_YAW;
        settled_ = 0;
        rotating_ = true;
        escaping_ = false;
        ROS_INFO("nav_to_pose: within goal radius (%.2fm) -> ROTATE_YAW", dist);
        tgt_vx = tgt_vy = tgt_vth = 0.0;
      }
      else
      {
        // goal direction expressed in the base frame (heading fixed)
        double c = cos(pose_yaw_), s = sin(pose_yaw_);
        double dx_b = c * dx + s * dy;
        double dy_b = -s * dx + c * dy;

        double v_att_x = 0.0, v_att_y = 0.0;
        if (dist > 0.05)
        {
          double mag = kp_trans_ * dist;
          if (dist < slow_radius_)
            mag *= dist / slow_radius_;
          mag = std::min(mag, max_vel_trans_);
          v_att_x = mag * dx_b / dist;
          v_att_y = mag * dy_b / dist;
        }

        // repulsion: pure translational obstacle avoidance from /scan
        double v_rep_x = 0.0, v_rep_y = 0.0;
        if (have_scan_)
        {
          for (size_t i = 0; i < scan_.ranges.size(); ++i)
          {
            double r = scan_.ranges[i];
            if (std::isnan(r) || r < scan_.range_min || r > scan_.range_max)
              continue;
            if (r < repulse_range_)
            {
              double a = scan_.angle_min + i * scan_.angle_increment;
              double w = (repulse_range_ - r) / repulse_range_;
              v_rep_x += w * cos(a);
              v_rep_y += w * sin(a);
            }
          }
          double mag = hypot(v_rep_x, v_rep_y);
          if (mag > 1.0)
          {
            v_rep_x /= mag;
            v_rep_y /= mag;
          }
          v_rep_x *= -repulse_gain_ * max_vel_trans_;
          v_rep_y *= -repulse_gain_ * max_vel_trans_;
        }

        // head-on obstacle: within +/-escape_angle of forward, closer than
        // escape_range -> start/stay in escape; the escape ends when the
        // forward cone is clear out to escape_resume_clear (a short resume
        // margin, not a long look-ahead: inside a small room the walls are
        // always in sight and a 2.5 m cone would never clear)
        bool head_on = false;
        bool cone_clear = true;
        if (have_scan_)
        {
          for (size_t i = 0; i < scan_.ranges.size(); ++i)
          {
            double r = scan_.ranges[i];
            if (std::isnan(r) || r < scan_.range_min || r > scan_.range_max)
              continue;
            double a = scan_.angle_min + i * scan_.angle_increment;
            if (fabs(a) >= escape_angle_)
              continue;
            if (r < escape_range_)
              head_on = true;
            if (r < escape_resume_clear_)
              cone_clear = false;
          }
        }

        // lateral headway in the current escape direction (heading locked,
        // so it is measured against the escape start pose); odom rather than
        // amcl, since the escape is a local motion primitive and odom is
        // noise-free over this short window
        double escape_lat = 0.0;
        if (escaping_ && have_odom_)
        {
          double s = sin(escape_start_yaw_), c = cos(escape_start_yaw_);
          escape_lat = escape_dir_ * (-s * (odom_x_ - escape_start_x_) +
                                      c * (odom_y_ - escape_start_y_));
          lat_hist_.push_back(std::make_pair(now, escape_lat));
          while (!lat_hist_.empty() &&
                 (now - lat_hist_.front().first).toSec() > escape_stuck_time_)
            lat_hist_.pop_front();
        }

        if (!escaping_ && head_on)
        {
          escaping_ = true;
          escape_dir_ = -1.0; // right (starboard) first
          switched_left_ = false;
          escape_start_ = now;
          lat_hist_.clear();
          lat_hist_.push_back(std::make_pair(now, 0.0));
          escape_start_x_ = have_odom_ ? odom_x_ : pose_x_;
          escape_start_y_ = have_odom_ ? odom_y_ : pose_y_;
          escape_start_yaw_ = have_odom_ ? odom_yaw_ : pose_yaw_;
          cmd_vx_ = 0.0; // hard stop the forward motion, then strafe
          ROS_INFO("nav_to_pose: obstacle head-on, escaping right");
        }
        else if (escaping_)
        {
          bool escape_stuck = lateralStalled(now, escape_lat);
          bool escape_timeout =
              (now - escape_start_).toSec() > escape_max_time_;
          if (cone_clear &&
              (escape_lat >= escape_min_progress_ || escape_stuck ||
               escape_timeout))
          {
            // front cone is drivable again (or the right escape is stuck/
            // timed out while the cone is clear) -> resume forward; never
            // switch direction when the path ahead is actually clear
            escaping_ = false;
            best_dist_ = dist;
            best_time_ = now;
            ROS_INFO("nav_to_pose: head-on obstacle cleared, resuming");
          }
          else if (!switched_left_ && !cone_clear &&
                   (escape_timeout || escape_stuck))
          {
            ROS_INFO("nav_to_pose: escape right stuck, switching left");
            escape_dir_ = 1.0;
            switched_left_ = true;
            escape_start_ = now;
            lat_hist_.clear();
            lat_hist_.push_back(std::make_pair(now, 0.0));
            escape_start_x_ = have_odom_ ? odom_x_ : pose_x_;
            escape_start_y_ = have_odom_ ? odom_y_ : pose_y_;
            escape_start_yaw_ = have_odom_ ? odom_yaw_ : pose_yaw_;
          }
        }

        double vx = v_att_x + v_rep_x;
        double vy = v_att_y + v_rep_y;
        if (escaping_)
        {
          // pure lateral escape: attraction dropped, and the along-heading
          // position is held (vx = 0) so the front obstacle's repulsion
          // cannot push the robot back out to the resume-clear range and
          // fake a clear (that would pogo: resume -> approach -> re-trigger
          // at a wall, with zero net progress); the cone then only clears
          // by genuinely strafing past the obstacle
          vx = 0.0;
          vy = v_rep_y + escape_dir_ * escape_speed_;
        }
        double vm = hypot(vx, vy);
        if (vm > max_vel_trans_)
        {
          vx *= max_vel_trans_ / vm;
          vy *= max_vel_trans_ / vm;
        }

        tgt_vx = vx;
        tgt_vy = vy;
        tgt_vth = 0.0; // never rotate while translating

        // stall detection
        if (best_dist_ < 0 || dist < best_dist_ - 0.005)
        {
          best_dist_ = dist;
          best_time_ = now;
        }
        // while escaping, lateral headway counts as progress; only a
        // left-escape whose headway has stalled is treated as a stall
        bool escape_stalled = escaping_ && switched_left_ &&
                              lateralStalled(now, escape_lat);
        if ((now - best_time_).toSec() > stall_time_ &&
            best_dist_ - dist < stall_min_progress_ && dist > 0.3 &&
            (!escaping_ || escape_stalled))
        {
          ROS_WARN("nav_to_pose: no progress for %.0fs, aborting", stall_time_);
          stop("stalled");
          if (as_.isActive())
            as_.setAborted();
          return;
        }
      }
    }
    else if (state_ == ROTATE_YAW)
    {
      double dist = hypot(goal_x_ - pose_x_, goal_y_ - pose_y_);
      if (dist > goal_radius_exit_)
      {
        // drifted too far: back to translation (band hysteresis)
        state_ = TRANSLATE;
        settled_ = 0;
        best_dist_ = dist;
        best_time_ = now;
        ROS_INFO("nav_to_pose: drifted to %.2fm from goal -> TRANSLATE", dist);
      }
      else
      {
        double err = angles::shortest_angular_distance(pose_yaw_, goal_yaw_);
        // tolerance band: stop within narrow bound, resume only beyond wide bound
        if (fabs(err) > goal_yaw_start_tolerance_)
          rotating_ = true;
        else if (fabs(err) < goal_yaw_tolerance_)
          rotating_ = false;

        if (rotating_)
        {
          settled_ = 0;
          tgt_vth = clamp(kp_rot_ * err, max_vel_theta_);
        }
        else
        {
          tgt_vth = 0.0;
          if (++settled_ >= settled_cycles_)
          {
            state_ = DONE;
            ROS_INFO("nav_to_pose: goal yaw reached (err=%.2fdeg) -> DONE",
                     err * 180.0 / M_PI);
            publishZero();
            if (as_.isActive())
            {
              move_base_msgs::MoveBaseResult res;
              as_.setSucceeded(res);
            }
            return;
          }
        }
      }
    }

    cmd_vx_ = ramp(cmd_vx_, tgt_vx, acc_lim_trans_, dt);
    cmd_vy_ = ramp(cmd_vy_, tgt_vy, acc_lim_trans_, dt);
    cmd_vth_ = ramp(cmd_vth_, tgt_vth, acc_lim_theta_, dt);

    geometry_msgs::Twist cmd;
    cmd.linear.x = cmd_vx_;
    cmd.linear.y = cmd_vy_;
    cmd.angular.z = cmd_vth_;
    cmd_pub_.publish(cmd);

    if (as_.isActive())
    {
      move_base_msgs::MoveBaseFeedback fb;
      fb.base_position.header.stamp = now;
      fb.base_position.header.frame_id = "map";
      fb.base_position.pose.position.x = pose_x_;
      fb.base_position.pose.position.y = pose_y_;
      tf2::Quaternion q;
      q.setRPY(0, 0, pose_yaw_);
      fb.base_position.pose.orientation = tf2::toMsg(q);
      as_.publishFeedback(fb);
    }
  }

  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;
  ros::Publisher cmd_pub_;
  ros::Subscriber pose_sub_, odom_sub_, scan_sub_, goal_sub_;

  sensor_msgs::LaserScan scan_;
  bool have_scan_ = false;
  bool have_pose_ = false;
  bool have_odom_ = false;
  bool have_goal_ = false;
  bool rotating_ = true;

  State state_ = IDLE;
  double pose_x_ = 0, pose_y_ = 0, pose_yaw_ = 0;
  double odom_x_ = 0, odom_y_ = 0, odom_yaw_ = 0;
  double goal_x_ = 0, goal_y_ = 0, goal_yaw_ = 0;
  double cmd_vx_ = 0, cmd_vy_ = 0, cmd_vth_ = 0;
  int settled_ = 0;
  double best_dist_ = -1.0;
  ros::Time best_time_;

  double max_vel_trans_, max_vel_theta_;
  double acc_lim_trans_, acc_lim_theta_;
  double kp_rot_, kp_trans_;
  double slow_radius_;
  double align_tolerance_, align_start_tolerance_;
  double goal_radius_, goal_radius_exit_;
  double goal_yaw_tolerance_, goal_yaw_start_tolerance_;
  int settled_cycles_;
  std::string scan_topic_;
  double repulse_range_, repulse_gain_;
  double escape_angle_, escape_range_, escape_resume_clear_, escape_speed_;
  double escape_stuck_time_, escape_max_time_, escape_min_progress_;
  double stall_time_, stall_min_progress_;
  double rate_;

  bool escaping_ = false;
  bool switched_left_ = false;
  double escape_dir_ = -1.0;
  ros::Time escape_start_;
  double escape_start_x_ = 0.0, escape_start_y_ = 0.0, escape_start_yaw_ = 0.0;
  std::deque<std::pair<ros::Time, double>> lat_hist_;

  // escape stalled when the lateral headway over the last escape_stuck_time_
  // window is below escape_min_progress_ (a window comparison rather than a
  // peak, so wall-bounce limit cycles also count as stalled)
  bool lateralStalled(const ros::Time& now, double escape_lat) const
  {
    if (lat_hist_.empty())
      return false;
    if ((now - lat_hist_.front().first).toSec() < escape_stuck_time_ - 0.05)
      return false; // window not full yet
    return (escape_lat - lat_hist_.front().second) < escape_min_progress_;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_to_pose");
  NavToPose node;
  node.spin();
  return 0;
}
