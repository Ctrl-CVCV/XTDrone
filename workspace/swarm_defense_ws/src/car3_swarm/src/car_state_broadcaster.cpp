#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>

// 1.6: read gazebo ground truth (/gazebo/model_states) and publish each car's
// state on /carN/shared_pose (Odometry): map-frame position + heading quaternion
// + world-frame linear/angular velocity, so every car can know all cars' states.

class CarStateBroadcaster
{
public:
  CarStateBroadcaster()
    : pnh_("~")
  {
    pnh_.param("rate", rate_, 20.0);
    pnh_.getParam("cars", carNames_);
    if (carNames_.empty())
    {
      ROS_FATAL("car_state_broadcaster: no 'cars' param (e.g. [car0, car1, car2])");
      ros::shutdown();
      return;
    }

    // sized up front: the timer can fire before the first model_states
    // callback, and publish() must never index empty vectors
    poses_.assign(carNames_.size(), geometry_msgs::Pose());
    twists_.assign(carNames_.size(), geometry_msgs::Twist());
    found_.assign(carNames_.size(), false);

    sub_ = nh_.subscribe("/gazebo/model_states", 10, &CarStateBroadcaster::cb, this);
    for (const auto& name : carNames_)
      pubs_.push_back(nh_.advertise<nav_msgs::Odometry>("/" + name + "/shared_pose", 10));
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &CarStateBroadcaster::publish, this);

    ROS_INFO("car_state_broadcaster: publishing shared_pose for %zu cars at %.1f Hz",
             carNames_.size(), rate_);
  }

  void cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    std::vector<geometry_msgs::Pose> poses(carNames_.size());
    std::vector<geometry_msgs::Twist> twists(carNames_.size());
    std::vector<bool> found(carNames_.size(), false);
    for (size_t k = 0; k < msg->name.size(); ++k)
    {
      for (size_t i = 0; i < carNames_.size(); ++i)
      {
        if (msg->name[k] == carNames_[i])
        {
          poses[i] = msg->pose[k];
          twists[i] = msg->twist[k];
          found[i] = true;
        }
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    poses_ = poses;
    twists_ = twists;
    found_ = found;
  }

  void publish(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < carNames_.size(); ++i)
    {
      if (!found_[i]) continue;

      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = "map";
      odom.child_frame_id = carNames_[i] + "/base_footprint";
      odom.pose.pose = poses_[i];
      odom.pose.pose.position.z = 0.0;
      odom.twist.twist = twists_[i];
      odom.twist.twist.linear.z = 0.0;
      odom.twist.twist.angular.x = 0.0;
      odom.twist.twist.angular.y = 0.0;
      pubs_[i].publish(odom);
    }
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  std::vector<ros::Publisher> pubs_;
  ros::Timer timer_;
  std::vector<std::string> carNames_;
  double rate_;
  std::mutex mutex_;
  std::vector<geometry_msgs::Pose> poses_;
  std::vector<geometry_msgs::Twist> twists_;
  std::vector<bool> found_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_state_broadcaster");
  CarStateBroadcaster node;
  ros::spin();
  return 0;
}
