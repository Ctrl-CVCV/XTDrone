#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

// Phase 3: publish /odom + odom->base_footprint TF from Gazebo ground truth
// (task requires ground-truth odometry, not wheel odometry).
// Multi-car ready: relative topic names (namespaced under /carN) and
// model_name/odom_frame/base_frame params (defaults keep single-car behavior).

class GroundTruthOdom
{
public:
  GroundTruthOdom()
    : nh_(), pnh_("~")
  {
    pnh_.param("model_name", model_name_, std::string("car3"));
    pnh_.param("odom_frame", odom_frame_, std::string("odom"));
    pnh_.param("base_frame", base_frame_, std::string("base_footprint"));

    sub_ = nh_.subscribe("/gazebo/model_states", 10, &GroundTruthOdom::cb, this);
    pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    timer_ = nh_.createTimer(ros::Duration(0.02), &GroundTruthOdom::publish, this);
  }

  void cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    int i = -1;
    for (size_t k = 0; k < msg->name.size(); ++k)
      if (msg->name[k] == model_name_) { i = static_cast<int>(k); break; }
    if (i < 0) return;
    latest_stamp_ = ros::Time::now();
    latest_pose_ = msg->pose[i];
    latest_twist_ = msg->twist[i];
  }

  void publish(const ros::TimerEvent&)
  {
    if (latest_stamp_.isZero()) return;

    nav_msgs::Odometry odom;
    odom.header.stamp = latest_stamp_;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = latest_pose_.position.x;
    odom.pose.pose.position.y = latest_pose_.position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = latest_pose_.orientation;
    odom.twist.twist.linear.x = latest_twist_.linear.x;
    odom.twist.twist.linear.y = latest_twist_.linear.y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = latest_twist_.angular.z;
    const double cv = 1e-4;
    for (int k = 0; k < 36; ++k) odom.pose.covariance[k] = 0.0;
    for (int k = 0; k < 36; ++k) odom.twist.covariance[k] = 0.0;
    for (int k = 0; k < 6; ++k) odom.pose.covariance[k * 7] = cv;
    for (int k = 0; k < 6; ++k) odom.twist.covariance[k * 7] = cv;
    pub_.publish(odom);

    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = latest_pose_.position.x;
    tf.transform.translation.y = latest_pose_.position.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = latest_pose_.orientation;
    tf_br_.sendTransform(tf);
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tf_br_;
  ros::Time latest_stamp_;
  geometry_msgs::Pose latest_pose_;
  geometry_msgs::Twist latest_twist_;
  std::string model_name_, odom_frame_, base_frame_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_truth_odom");
  GroundTruthOdom node;
  ros::spin();
  return 0;
}
