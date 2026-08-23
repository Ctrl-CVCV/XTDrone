#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <mutex>
#include <map>
#include <algorithm>

// 1.7: inject the other cars' virtual boxes (2D rectangles at their ground-truth
// centers) into this car's scan, so the high-mounted lidar can "see" other cars
// and nav_to_pose's avoidance reacts to them.
//
// Runs inside /carN: subscribes "scan" + "odom" (relative) and each other car's
// "/carM/shared_pose" (absolute), publishes "scan_filtered". Poses are world-frame
// ground truth (odom frame == world frame == map frame), so no TF lookups are
// needed. Without other-car data the node is a transparent passthrough.
//
// Phase 7: per-role box sizing. Cars use the smaller target_safe box for the
// intruder (intruder_name, defaults to car2) so defenders can close to capture
// distance without model penetration; friendly-friendy pairs keep the normal box.

class VirtualObstacleNode
{
public:
  VirtualObstacleNode()
    : pnh_("~")
  {
    pnh_.param("box_x", boxX_, 0.45);
    pnh_.param("box_y", boxY_, 0.38);
    pnh_.param("target_box_x", targetBoxX_, 0.34);
    pnh_.param("target_box_y", targetBoxY_, 0.30);
    pnh_.param("obstacle_timeout", obstacleTimeout_, 0.5);
    if (!pnh_.getParam("intruder_names", intruderNames_))
    {
      std::string legacyIntruder;
      pnh_.param("intruder_name", legacyIntruder, std::string("car2"));
      intruderNames_.push_back(legacyIntruder);
    }
    pnh_.getParam("other_cars", otherCars_);
    hx_ = boxX_ / 2.0;
    hy_ = boxY_ / 2.0;
    targetHx_ = targetBoxX_ / 2.0;
    targetHy_ = targetBoxY_ / 2.0;

    subScan_ = nh_.subscribe("scan", 10, &VirtualObstacleNode::scanCb, this);
    subOdom_ = nh_.subscribe("odom", 10, &VirtualObstacleNode::odomCb, this);
    for (const auto& name : otherCars_)
      subOthers_.push_back(nh_.subscribe<nav_msgs::Odometry>(
          "/" + name + "/shared_pose", 10,
          boost::bind(&VirtualObstacleNode::otherCb, this, _1, name)));
    pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_filtered", 10);

    ROS_INFO("virtual_obstacle_node: friendly box %.2fx%.2f, target box "
             "%.2fx%.2f for %zu intruders, injecting %zu other cars, timeout %.2fs",
             boxX_, boxY_, targetBoxX_, targetBoxY_, intruderNames_.size(),
             otherCars_.size(), obstacleTimeout_);
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ownPose_ = poseOf(msg->pose.pose);
    haveOwn_ = true;
  }

  void otherCb(const nav_msgs::Odometry::ConstPtr& msg, const std::string& name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    otherPoses_[name] = poseOf(msg->pose.pose);
    otherStamps_[name] = ros::Time::now();
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    sensor_msgs::LaserScan out = *msg;

    std::vector<Box> boxes;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!haveOwn_) { pub_.publish(out); return; }
      const ros::Time now = ros::Time::now();
      for (const auto& name : otherCars_)
      {
        auto it = otherPoses_.find(name);
        auto stampIt = otherStamps_.find(name);
        if (it == otherPoses_.end() || stampIt == otherStamps_.end()) continue;
        if ((now - stampIt->second).toSec() > obstacleTimeout_) continue;
        Box b;
        b.pose = relative(ownPose_, it->second);
        if (std::find(intruderNames_.begin(), intruderNames_.end(), name) != intruderNames_.end())
        {
          b.hx = targetHx_;
          b.hy = targetHy_;
        }
        else
        {
          b.hx = hx_;
          b.hy = hy_;
        }
        boxes.push_back(b);
      }
    }
    if (boxes.empty()) { pub_.publish(out); return; }

    const double a0 = msg->angle_min;
    const double inc = msg->angle_increment;
    const double rmin = msg->range_min;
    for (size_t i = 0; i < out.ranges.size(); ++i)
    {
      const double th = a0 + inc * i;
      const double ux = std::cos(th), uy = std::sin(th);
      double r = out.ranges[i];
      for (const auto& box : boxes)
      {
        const double d = hitDistance(box, ux, uy);
        if (d >= 0.0 && d < r)
          r = d > 0.0 ? d : rmin;
      }
      out.ranges[i] = r;
    }
    pub_.publish(out);
  }

private:
  struct Pose2 { double x, y, yaw; };
  struct Box { Pose2 pose; double hx, hy; };

  static Pose2 poseOf(const geometry_msgs::Pose& p)
  {
    Pose2 o;
    o.x = p.position.x;
    o.y = p.position.y;
    o.yaw = tf2::getYaw(p.orientation);
    return o;
  }

  // pose of other in my (lidar) frame
  static Pose2 relative(const Pose2& me, const Pose2& other)
  {
    const double c = std::cos(-me.yaw), s = std::sin(-me.yaw);
    const double dx = other.x - me.x, dy = other.y - me.y;
    Pose2 r;
    r.x = c * dx - s * dy;
    r.y = s * dx + c * dy;
    r.yaw = other.yaw - me.yaw;
    return r;
  }

  // distance from lidar origin (0,0) along unit dir (ux,uy) to the box; -1 = miss
  double hitDistance(const Box& box, double ux, double uy) const
  {
    const double hx = box.hx, hy = box.hy;
    const double c = std::cos(-box.pose.yaw), s = std::sin(-box.pose.yaw);
    // lidar origin in box frame
    const double px = -(c * box.pose.x - s * box.pose.y);
    const double py = -(s * box.pose.x + c * box.pose.y);
    // beam direction in box frame
    const double dx = c * ux - s * uy;
    const double dy = s * ux + c * uy;

    double tEnter = -1e9, tExit = 1e9;
    const double eps = 1e-9;

    if (std::abs(dx) < eps)
    {
      if (px < -hx || px > hx) return -1.0;
    }
    else
    {
      double t1 = (-hx - px) / dx, t2 = (hx - px) / dx;
      if (t1 > t2) std::swap(t1, t2);
      tEnter = std::max(tEnter, t1);
      tExit = std::min(tExit, t2);
    }
    if (std::abs(dy) < eps)
    {
      if (py < -hy || py > hy) return -1.0;
    }
    else
    {
      double t1 = (-hy - py) / dy, t2 = (hy - py) / dy;
      if (t1 > t2) std::swap(t1, t2);
      tEnter = std::max(tEnter, t1);
      tExit = std::min(tExit, t2);
    }

    if (tEnter < tExit && tExit > 0.0)
      return tEnter;
    return -1.0;
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber subScan_, subOdom_;
  std::vector<ros::Subscriber> subOthers_;
  ros::Publisher pub_;
  std::vector<std::string> otherCars_;
  std::vector<std::string> intruderNames_;
  double boxX_, boxY_, targetBoxX_, targetBoxY_, obstacleTimeout_;
  double hx_, hy_, targetHx_, targetHy_;
  std::mutex mutex_;
  bool haveOwn_ = false;
  Pose2 ownPose_;
  std::map<std::string, Pose2> otherPoses_;
  std::map<std::string, ros::Time> otherStamps_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "virtual_obstacle_node");
  VirtualObstacleNode node;
  ros::spin();
  return 0;
}
