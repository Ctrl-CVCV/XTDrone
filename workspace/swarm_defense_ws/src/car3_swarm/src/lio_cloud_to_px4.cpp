#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

// Convert a SWARM-LIO registered cloud into the PX4 EKF local frame.
//
// The cloud and LIO odometry are both expressed in quadN/world.  PX4's
// local_position/odom is expressed in map.  At startup, while the simulated
// vehicle is stationary, estimate the fixed map <- quadN/world transform from
// the two poses.  The transform is then latched; it is deliberately not
// recomputed while flying, because doing so would turn EKF/LIO estimation
// differences into a moving obstacle map.
class LioCloudToPx4
{
public:
  LioCloudToPx4(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("cloud_in", cloud_in_, "/quad0/cloud_registered");
    pnh.param<std::string>("lio_odom", lio_odom_topic_, "/quad0/lidar_slam/odom");
    pnh.param<std::string>("px4_odom", px4_odom_topic_, "/iris_0/mavros/local_position/odom");
    pnh.param<std::string>("cloud_out", cloud_out_, "/iris_0/cloud_registered_ekf");
    pnh.param<std::string>("output_frame", output_frame_, "map");
    pnh.param<int>("calibration_samples", calibration_samples_, 30);
    pnh.param<double>("max_pair_age", max_pair_age_, 0.20);

    cloud_sub_ = nh.subscribe(cloud_in_, 2, &LioCloudToPx4::cloudCallback, this);
    lio_sub_ = nh.subscribe(lio_odom_topic_, 20, &LioCloudToPx4::lioCallback, this);
    px4_sub_ = nh.subscribe(px4_odom_topic_, 50, &LioCloudToPx4::px4Callback, this);
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_out_, 2);

    ROS_INFO("LIO cloud adapter: %s + %s -> %s [%s], calibration=%d",
             cloud_in_.c_str(), lio_odom_topic_.c_str(), cloud_out_.c_str(),
             output_frame_.c_str(), calibration_samples_);
  }

private:
  struct PoseSample
  {
    ros::Time stamp;
    Eigen::Isometry3d lio;
    Eigen::Isometry3d px4;
  };

  static Eigen::Isometry3d pose(const geometry_msgs::Pose& p)
  {
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x,
                         p.orientation.y, p.orientation.z);
    if (!std::isfinite(q.w()) || !std::isfinite(q.x()) ||
        !std::isfinite(q.y()) || !std::isfinite(q.z()) || q.norm() < 1e-8)
      q = Eigen::Quaterniond::Identity();
    else
      q.normalize();
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = q.toRotationMatrix();
    t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    return t;
  }

  void lioCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lio_ = pose(msg->pose.pose);
    lio_stamp_ = msg->header.stamp;
    have_lio_ = true;
    tryCalibrateLocked();
  }

  void px4Callback(const nav_msgs::OdometryConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    px4_ = pose(msg->pose.pose);
    px4_stamp_ = msg->header.stamp;
    have_px4_ = true;
    tryCalibrateLocked();
  }

  void tryCalibrateLocked()
  {
    if (calibrated_ || !have_lio_ || !have_px4_)
      return;
    const double age = std::fabs((lio_stamp_ - px4_stamp_).toSec());
    if (age > max_pair_age_)
      return;

    // map <- quadN/world = (map <- body) * (quadN/world <- body)^-1
    const Eigen::Isometry3d candidate = px4_ * lio_.inverse();
    if (!candidate.matrix().allFinite())
      return;
    samples_.push_back(candidate);
    if (static_cast<int>(samples_.size()) < calibration_samples_)
    {
      ROS_INFO_THROTTLE(1.0, "calibrating %s -> %s: %zu/%d samples",
                        lio_odom_topic_.c_str(), output_frame_.c_str(),
                        samples_.size(), calibration_samples_);
      return;
    }

    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond qsum(0.0, 0.0, 0.0, 0.0);
    Eigen::Quaterniond qref(samples_[0].rotation());
    for (const auto& sample : samples_)
    {
      translation += sample.translation();
      Eigen::Quaterniond q(sample.rotation());
      if (q.dot(qref) < 0.0)
        q.coeffs() *= -1.0;
      qsum.coeffs() += q.coeffs();
    }
    translation /= static_cast<double>(samples_.size());
    qsum.normalize();
    transform_ = Eigen::Isometry3d::Identity();
    transform_.linear() = qsum.toRotationMatrix();
    transform_.translation() = translation;
    calibrated_ = true;
    ROS_INFO("calibration complete: t=[%.4f %.4f %.4f], q=[%.5f %.5f %.5f %.5f]",
             translation.x(), translation.y(), translation.z(),
             qsum.x(), qsum.y(), qsum.z(), qsum.w());
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    Eigen::Isometry3d transform;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!calibrated_)
      {
        ROS_WARN_THROTTLE(2.0, "waiting for fixed LIO -> PX4 cloud calibration");
        return;
      }
      transform = transform_;
    }

    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(*msg, input);
    pcl::PointCloud<pcl::PointXYZ> output;
    output.reserve(input.size());
    for (const auto& in : input.points)
    {
      if (!std::isfinite(in.x) || !std::isfinite(in.y) || !std::isfinite(in.z))
        continue;
      const Eigen::Vector3d p = transform * Eigen::Vector3d(in.x, in.y, in.z);
      pcl::PointXYZ out;
      out.x = static_cast<float>(p.x());
      out.y = static_cast<float>(p.y());
      out.z = static_cast<float>(p.z());
      output.push_back(out);
    }
    output.width = static_cast<uint32_t>(output.size());
    output.height = 1;
    output.is_dense = true;

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(output, out_msg);
    out_msg.header = msg->header;
    out_msg.header.frame_id = output_frame_;
    cloud_pub_.publish(out_msg);
  }

  ros::Subscriber cloud_sub_, lio_sub_, px4_sub_;
  ros::Publisher cloud_pub_;
  std::string cloud_in_, lio_odom_topic_, px4_odom_topic_, cloud_out_, output_frame_;
  int calibration_samples_;
  double max_pair_age_;
  std::mutex mutex_;
  bool have_lio_{false}, have_px4_{false}, calibrated_{false};
  ros::Time lio_stamp_, px4_stamp_;
  Eigen::Isometry3d lio_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d px4_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d transform_{Eigen::Isometry3d::Identity()};
  std::vector<Eigen::Isometry3d> samples_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lio_cloud_to_px4");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  LioCloudToPx4 node(nh, pnh);
  ros::spin();
  return 0;
}
