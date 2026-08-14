#include <depthimage_to_laserscan/GravityAlignedDepthToLaserScan.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <boost/thread/mutex.hpp>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace depthimage_to_laserscan
{
namespace
{

const double kPi = 3.14159265358979323846;

std::string numberString(double value, int precision = 4)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

void addDiagnosticValue(
    diagnostic_msgs::DiagnosticStatus* status,
    const std::string& key,
    const std::string& value)
{
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  status->values.push_back(item);
}

}  // namespace

class GravityAlignedDepthToLaserScanNode
{
public:
  GravityAlignedDepthToLaserScanNode()
    : nh_()
    , pnh_("~")
    , image_transport_(nh_)
    , imu_timeout_(0.12)
    , require_imu_(true)
    , diagnostic_period_(1.0)
  {
    GravityAlignedFilterConfig config;
    double mount_roll_degrees = 0.0;
    double mount_pitch_degrees = 18.8282205;
    double mount_yaw_degrees = 0.0;
    double max_ground_tilt_degrees = 20.0;
    pnh_.param("camera_mount_roll_deg", mount_roll_degrees, mount_roll_degrees);
    pnh_.param("camera_mount_pitch_deg", mount_pitch_degrees, mount_pitch_degrees);
    pnh_.param("camera_mount_yaw_deg", mount_yaw_degrees, mount_yaw_degrees);
    pnh_.param("range_min", config.range_min, config.range_min);
    pnh_.param("range_max", config.range_max, config.range_max);
    pnh_.param("angle_min", config.angle_min, config.angle_min);
    pnh_.param("angle_max", config.angle_max, config.angle_max);
    pnh_.param("angle_increment", config.angle_increment, config.angle_increment);
    pnh_.param("scan_time", config.scan_time, config.scan_time);
    pnh_.param("depth_stride", config.depth_stride, config.depth_stride);
    pnh_.param("min_camera_height", config.min_camera_height, config.min_camera_height);
    pnh_.param("max_camera_height", config.max_camera_height, config.max_camera_height);
    pnh_.param("nominal_camera_height", config.nominal_camera_height, config.nominal_camera_height);
    pnh_.param("ground_distance_threshold", config.ground_distance_threshold,
               config.ground_distance_threshold);
    pnh_.param("min_obstacle_height", config.min_obstacle_height, config.min_obstacle_height);
    pnh_.param("max_obstacle_height", config.max_obstacle_height, config.max_obstacle_height);
    pnh_.param("max_ground_tilt_deg", max_ground_tilt_degrees, max_ground_tilt_degrees);
    pnh_.param("ransac_iterations", config.ransac_iterations, config.ransac_iterations);
    pnh_.param("min_ground_inliers", config.min_ground_inliers, config.min_ground_inliers);
    pnh_.param("min_ground_inlier_ratio", config.min_ground_inlier_ratio,
               config.min_ground_inlier_ratio);
    pnh_.param("max_ransac_points", config.max_ransac_points, config.max_ransac_points);
    pnh_.param("fallback_to_nominal_ground", config.fallback_to_nominal_ground,
               config.fallback_to_nominal_ground);
    pnh_.param("use_inf", config.use_inf, config.use_inf);
    pnh_.param("output_frame_id", config.output_frame_id, config.output_frame_id);
    pnh_.param("imu_timeout", imu_timeout_, imu_timeout_);
    pnh_.param("require_imu", require_imu_, require_imu_);
    pnh_.param("diagnostic_period", diagnostic_period_, diagnostic_period_);

    config.camera_mount_roll = mount_roll_degrees * kPi / 180.0;
    config.camera_mount_pitch = mount_pitch_degrees * kPi / 180.0;
    config.camera_mount_yaw = mount_yaw_degrees * kPi / 180.0;
    config.max_ground_tilt = max_ground_tilt_degrees * kPi / 180.0;
    converter_.setConfig(config);

    scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
    clearing_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("clearing_scan", 10);
    diagnostics_publisher_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 2);
    imu_subscriber_ = nh_.subscribe("imu", 50, &GravityAlignedDepthToLaserScanNode::imuCallback, this);
    depth_subscriber_ = image_transport_.subscribeCamera(
        "image", 2, &GravityAlignedDepthToLaserScanNode::depthCallback, this,
        image_transport::TransportHints("raw", ros::TransportHints(), pnh_));

    ROS_INFO_STREAM(
        "gravity-aligned depth filter ready: output_frame=" << config.output_frame_id
        << ", mount_rpy_deg=[" << mount_roll_degrees << ", " << mount_pitch_degrees
        << ", " << mount_yaw_degrees << "], stride=" << config.depth_stride
        << ", range=[" << config.range_min << ", " << config.range_max << "]");
  }

private:
  bool diagnosticDue()
  {
    const ros::Time now = ros::Time::now();
    if (!last_diagnostic_time_.isZero() &&
        (now - last_diagnostic_time_).toSec() < diagnostic_period_)
    {
      return false;
    }
    last_diagnostic_time_ = now;
    return true;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& message)
  {
    boost::mutex::scoped_lock lock(imu_mutex_);
    latest_imu_ = message;
    latest_imu_receipt_time_ = ros::Time::now();
  }

  void publishFailureDiagnostic(
      const std_msgs::Header& header,
      const std::string& message,
      double imu_age)
  {
    if (!diagnosticDue())
    {
      return;
    }
    diagnostic_msgs::DiagnosticArray array;
    array.header = header;
    diagnostic_msgs::DiagnosticStatus status;
    status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    status.name = ros::this_node::getName() + "/ground_filter";
    status.hardware_id = "d435i_depth_imu";
    status.message = message;
    addDiagnosticValue(&status, "imu_age_sec", numberString(imu_age));
    array.status.push_back(status);
    diagnostics_publisher_.publish(array);
  }

  void publishStatsDiagnostic(
      const std_msgs::Header& header,
      const GravityAlignedFilterStats& stats,
      double imu_age)
  {
    if (!diagnosticDue())
    {
      return;
    }
    diagnostic_msgs::DiagnosticArray array;
    array.header = header;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = ros::this_node::getName() + "/ground_filter";
    status.hardware_id = "d435i_depth_imu";
    if (stats.ground_plane_detected)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "gravity-aligned ground plane detected";
    }
    else if (stats.used_nominal_ground)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "ground RANSAC failed; using nominal camera height";
    }
    else
    {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "ground RANSAC failed; scan contains no filtered obstacles";
    }
    addDiagnosticValue(&status, "imu_age_sec", numberString(imu_age));
    addDiagnosticValue(&status, "valid_depth_points", std::to_string(stats.valid_depth_points));
    addDiagnosticValue(&status, "ground_candidates", std::to_string(stats.ground_candidate_points));
    addDiagnosticValue(&status, "ground_inliers", std::to_string(stats.ground_inlier_points));
    addDiagnosticValue(&status, "ground_height_m", numberString(stats.ground_height));
    addDiagnosticValue(&status, "ground_tilt_deg", numberString(stats.ground_tilt * 180.0 / kPi));
    addDiagnosticValue(&status, "obstacle_points", std::to_string(stats.obstacle_points));
    addDiagnosticValue(&status, "observed_scan_bins", std::to_string(stats.observed_scan_bins));
    addDiagnosticValue(&status, "clearing_scan_bins", std::to_string(stats.clearing_scan_bins));
    array.status.push_back(status);
    diagnostics_publisher_.publish(array);
  }

  void depthCallback(
      const sensor_msgs::ImageConstPtr& depth_message,
      const sensor_msgs::CameraInfoConstPtr& info_message)
  {
    sensor_msgs::ImuConstPtr imu_message;
    ros::Time imu_receipt_time;
    {
      boost::mutex::scoped_lock lock(imu_mutex_);
      imu_message = latest_imu_;
      imu_receipt_time = latest_imu_receipt_time_;
    }

    if (!imu_message)
    {
      ROS_ERROR_THROTTLE(1.0, "Ground-filtered scan suppressed: no IMU message received.");
      publishFailureDiagnostic(depth_message->header, "no IMU message", -1.0);
      return;
    }
    double imu_age = 0.0;
    if (!depth_message->header.stamp.isZero() && !imu_message->header.stamp.isZero())
    {
      imu_age = std::fabs((depth_message->header.stamp - imu_message->header.stamp).toSec());
    }
    else
    {
      imu_age = (ros::Time::now() - imu_receipt_time).toSec();
    }
    if (require_imu_ && imu_age > imu_timeout_)
    {
      ROS_ERROR_THROTTLE(
          1.0, "Ground-filtered scan suppressed: IMU is %.3f s away from depth frame (limit %.3f s).",
          imu_age, imu_timeout_);
      publishFailureDiagnostic(depth_message->header, "IMU/depth timestamps are too far apart", imu_age);
      return;
    }
    if (imu_message->orientation_covariance[0] < 0.0)
    {
      ROS_ERROR_THROTTLE(1.0, "Ground-filtered scan suppressed: IMU orientation is unavailable.");
      publishFailureDiagnostic(depth_message->header, "IMU orientation unavailable", imu_age);
      return;
    }

    try
    {
      GravityAlignedFilterStats stats;
      sensor_msgs::LaserScanPtr clearing_scan;
      sensor_msgs::LaserScanPtr scan = converter_.convert(
          depth_message, info_message, *imu_message, &stats, &clearing_scan);
      scan_publisher_.publish(scan);
      clearing_scan_publisher_.publish(clearing_scan);
      publishStatsDiagnostic(depth_message->header, stats, imu_age);
      if (!stats.ground_plane_detected)
      {
        ROS_WARN_THROTTLE(
            1.0, "Ground RANSAC did not converge; nominal-height fallback=%s.",
            stats.used_nominal_ground ? "active" : "disabled");
      }
    }
    catch (const std::runtime_error& error)
    {
      ROS_ERROR_THROTTLE(1.0, "Could not create ground-filtered scan: %s", error.what());
      publishFailureDiagnostic(depth_message->header, error.what(), imu_age);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport image_transport_;
  image_transport::CameraSubscriber depth_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Publisher scan_publisher_;
  ros::Publisher clearing_scan_publisher_;
  ros::Publisher diagnostics_publisher_;
  boost::mutex imu_mutex_;
  sensor_msgs::ImuConstPtr latest_imu_;
  ros::Time latest_imu_receipt_time_;
  double imu_timeout_;
  bool require_imu_;
  double diagnostic_period_;
  ros::Time last_diagnostic_time_;
  GravityAlignedDepthToLaserScan converter_;
};

}  // namespace depthimage_to_laserscan

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gravity_aligned_depth_to_laserscan");
  try
  {
    depthimage_to_laserscan::GravityAlignedDepthToLaserScanNode node;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("Could not start gravity-aligned depth filter: %s", error.what());
    return 1;
  }
  return 0;
}
