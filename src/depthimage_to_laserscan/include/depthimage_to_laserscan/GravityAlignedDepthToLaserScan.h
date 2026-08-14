#ifndef GRAVITY_ALIGNED_DEPTH_TO_LASER_SCAN_H
#define GRAVITY_ALIGNED_DEPTH_TO_LASER_SCAN_H

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>

#include <cstddef>
#include <string>
#include <vector>

namespace depthimage_to_laserscan
{

struct GravityAlignedPoint
{
  GravityAlignedPoint() : x(0.0), y(0.0), z(0.0) {}
  GravityAlignedPoint(double x_value, double y_value, double z_value)
    : x(x_value), y(y_value), z(z_value) {}

  double x;
  double y;
  double z;
};

struct GravityAlignedFilterConfig
{
  GravityAlignedFilterConfig();

  double camera_mount_roll;
  double camera_mount_pitch;
  double camera_mount_yaw;

  double range_min;
  double range_max;
  double angle_min;
  double angle_max;
  double angle_increment;
  double scan_time;
  int depth_stride;

  double min_camera_height;
  double max_camera_height;
  double nominal_camera_height;
  double ground_distance_threshold;
  double min_obstacle_height;
  double max_obstacle_height;
  double max_ground_tilt;
  int ransac_iterations;
  int min_ground_inliers;
  double min_ground_inlier_ratio;
  int max_ransac_points;

  bool fallback_to_nominal_ground;
  bool use_inf;
  std::string output_frame_id;
};

struct GravityAlignedFilterStats
{
  GravityAlignedFilterStats();

  std::size_t valid_depth_points;
  std::size_t ground_candidate_points;
  std::size_t ground_inlier_points;
  std::size_t obstacle_points;
  std::size_t observed_scan_bins;
  bool ground_plane_detected;
  bool used_nominal_ground;
  double ground_height;
  double ground_tilt;
};

class GravityAlignedDepthToLaserScan
{
public:
  GravityAlignedDepthToLaserScan();

  void setConfig(const GravityAlignedFilterConfig& config);
  const GravityAlignedFilterConfig& getConfig() const;

  sensor_msgs::LaserScanPtr convert(
      const sensor_msgs::ImageConstPtr& depth_msg,
      const sensor_msgs::CameraInfoConstPtr& info_msg,
      const sensor_msgs::Imu& imu_msg,
      GravityAlignedFilterStats* stats = NULL) const;

  // Public to make the ground model independently testable with synthetic
  // gravity-aligned point clouds.
  sensor_msgs::LaserScanPtr convertLevelPoints(
      const std::vector<GravityAlignedPoint>& points,
      const std_msgs::Header& header,
      GravityAlignedFilterStats* stats = NULL) const;

private:
  struct Plane
  {
    Plane() : nx(0.0), ny(0.0), nz(1.0), d(0.0), valid(false) {}
    double nx;
    double ny;
    double nz;
    double d;
    bool valid;
  };

  void validateConfig() const;
  Plane estimateGroundPlane(
      const std::vector<GravityAlignedPoint>& points,
      GravityAlignedFilterStats* stats) const;
  bool refineGroundPlane(
      const std::vector<GravityAlignedPoint>& candidates,
      const std::vector<std::size_t>& inlier_indices,
      Plane* plane) const;
  double signedDistance(const Plane& plane, const GravityAlignedPoint& point) const;
  sensor_msgs::LaserScanPtr makeScan(const std_msgs::Header& header) const;

  GravityAlignedFilterConfig config_;
};

}  // namespace depthimage_to_laserscan

#endif
