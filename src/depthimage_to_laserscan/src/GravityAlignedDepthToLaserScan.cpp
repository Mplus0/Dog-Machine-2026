#include <depthimage_to_laserscan/GravityAlignedDepthToLaserScan.h>

#include <cmath>

#include <depthimage_to_laserscan/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace depthimage_to_laserscan
{
namespace
{

const double kPi = 3.14159265358979323846;

double clampValue(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

uint32_t nextRandom(uint32_t* state)
{
  *state = (*state * 1664525u) + 1013904223u;
  return *state;
}

bool solveThreeByThree(double matrix[3][4], double solution[3])
{
  for (int column = 0; column < 3; ++column)
  {
    int pivot = column;
    for (int row = column + 1; row < 3; ++row)
    {
      if (std::fabs(matrix[row][column]) > std::fabs(matrix[pivot][column]))
      {
        pivot = row;
      }
    }
    if (std::fabs(matrix[pivot][column]) < 1e-9)
    {
      return false;
    }
    if (pivot != column)
    {
      for (int entry = column; entry < 4; ++entry)
      {
        std::swap(matrix[column][entry], matrix[pivot][entry]);
      }
    }
    const double divisor = matrix[column][column];
    for (int entry = column; entry < 4; ++entry)
    {
      matrix[column][entry] /= divisor;
    }
    for (int row = 0; row < 3; ++row)
    {
      if (row == column)
      {
        continue;
      }
      const double factor = matrix[row][column];
      for (int entry = column; entry < 4; ++entry)
      {
        matrix[row][entry] -= factor * matrix[column][entry];
      }
    }
  }
  solution[0] = matrix[0][3];
  solution[1] = matrix[1][3];
  solution[2] = matrix[2][3];
  return true;
}

template<typename T>
void collectLevelPoints(
    const sensor_msgs::ImageConstPtr& depth_msg,
    const image_geometry::PinholeCameraModel& camera_model,
    const tf2::Matrix3x3& mount_rotation,
    const tf2::Matrix3x3& level_from_body,
    int stride,
    double maximum_depth,
    std::vector<GravityAlignedPoint>* points)
{
  const T* first_pixel = reinterpret_cast<const T*>(&depth_msg->data[0]);
  const int row_step = depth_msg->step / sizeof(T);
  const double unit_scale = DepthTraits<T>::toMeters(T(1));
  const double fx = camera_model.fx();
  const double fy = camera_model.fy();
  const double cx = camera_model.cx();
  const double cy = camera_model.cy();

  for (int v = 0; v < static_cast<int>(depth_msg->height); v += stride)
  {
    const T* row = first_pixel + v * row_step;
    for (int u = 0; u < static_cast<int>(depth_msg->width); u += stride)
    {
      const T raw_depth = row[u];
      if (!DepthTraits<T>::valid(raw_depth))
      {
        continue;
      }
      const double depth = DepthTraits<T>::toMeters(raw_depth);
      if (!std::isfinite(depth) || depth <= 0.0 || depth > maximum_depth)
      {
        continue;
      }

      // Rectified depth images use the ROS optical convention: +X right,
      // +Y down, +Z forward. Convert first to the REP-103 robot convention
      // (+X forward, +Y left, +Z up), then apply the calibrated camera mount
      // and the IMU-derived gravity alignment.
      const double optical_x = (static_cast<double>(u) - cx) * raw_depth * unit_scale / fx;
      const double optical_y = (static_cast<double>(v) - cy) * raw_depth * unit_scale / fy;
      const tf2::Vector3 nominal_body(depth, -optical_x, -optical_y);
      const tf2::Vector3 level_point = level_from_body * (mount_rotation * nominal_body);
      points->push_back(GravityAlignedPoint(level_point.x(), level_point.y(), level_point.z()));
    }
  }
}

}  // namespace

GravityAlignedFilterConfig::GravityAlignedFilterConfig()
  : camera_mount_roll(0.0)
  , camera_mount_pitch(18.8282205 * kPi / 180.0)
  , camera_mount_yaw(0.0)
  , range_min(0.1)
  , range_max(3.0)
  , angle_min(-0.75)
  , angle_max(0.75)
  , angle_increment(1.5 / 639.0)
  , scan_time(1.0 / 15.0)
  , depth_stride(2)
  , min_camera_height(0.20)
  , max_camera_height(0.80)
  , nominal_camera_height(0.415)
  , ground_distance_threshold(0.04)
  , min_obstacle_height(0.06)
  , max_obstacle_height(1.50)
  , max_ground_tilt(20.0 * kPi / 180.0)
  , ransac_iterations(40)
  , min_ground_inliers(200)
  , min_ground_inlier_ratio(0.08)
  , max_ransac_points(6000)
  , fallback_to_nominal_ground(true)
  , use_inf(false)
  , output_frame_id("scan_ground_filtered_link")
{
}

GravityAlignedFilterStats::GravityAlignedFilterStats()
  : valid_depth_points(0)
  , ground_candidate_points(0)
  , ground_inlier_points(0)
  , obstacle_points(0)
  , observed_scan_bins(0)
  , clearing_scan_bins(0)
  , ground_plane_detected(false)
  , used_nominal_ground(false)
  , ground_height(std::numeric_limits<double>::quiet_NaN())
  , ground_tilt(std::numeric_limits<double>::quiet_NaN())
{
}

GravityAlignedDepthToLaserScan::GravityAlignedDepthToLaserScan()
  : config_()
{
}

void GravityAlignedDepthToLaserScan::setConfig(const GravityAlignedFilterConfig& config)
{
  config_ = config;
  validateConfig();
}

const GravityAlignedFilterConfig& GravityAlignedDepthToLaserScan::getConfig() const
{
  return config_;
}

void GravityAlignedDepthToLaserScan::validateConfig() const
{
  if (config_.depth_stride < 1)
  {
    throw std::runtime_error("depth_stride must be at least 1");
  }
  if (!(config_.range_min >= 0.0 && config_.range_max > config_.range_min))
  {
    throw std::runtime_error("range_max must be greater than range_min");
  }
  if (!(config_.angle_max > config_.angle_min && config_.angle_increment > 0.0))
  {
    throw std::runtime_error("invalid scan angle limits or increment");
  }
  if (!(config_.min_camera_height > 0.0 &&
        config_.max_camera_height > config_.min_camera_height &&
        config_.nominal_camera_height >= config_.min_camera_height &&
        config_.nominal_camera_height <= config_.max_camera_height))
  {
    throw std::runtime_error("invalid camera height limits");
  }
  if (!(config_.ground_distance_threshold > 0.0 &&
        config_.min_obstacle_height >= config_.ground_distance_threshold &&
        config_.max_obstacle_height > config_.min_obstacle_height))
  {
    throw std::runtime_error("invalid ground or obstacle height limits");
  }
  if (!(config_.max_ground_tilt > 0.0 && config_.max_ground_tilt < kPi / 2.0))
  {
    throw std::runtime_error("max_ground_tilt must be between 0 and pi/2");
  }
  if (config_.ransac_iterations < 1 || config_.min_ground_inliers < 3 ||
      config_.max_ransac_points < config_.min_ground_inliers)
  {
    throw std::runtime_error("invalid RANSAC limits");
  }
  if (!(config_.min_ground_inlier_ratio > 0.0 && config_.min_ground_inlier_ratio <= 1.0))
  {
    throw std::runtime_error("min_ground_inlier_ratio must be in (0, 1]");
  }
}

sensor_msgs::LaserScanPtr GravityAlignedDepthToLaserScan::convert(
    const sensor_msgs::ImageConstPtr& depth_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg,
    const sensor_msgs::Imu& imu_msg,
    GravityAlignedFilterStats* stats,
    sensor_msgs::LaserScanPtr* clearing_scan) const
{
  validateConfig();
  if (!depth_msg || !info_msg)
  {
    throw std::runtime_error("depth image and camera info are required");
  }
  if (depth_msg->width == 0 || depth_msg->height == 0)
  {
    throw std::runtime_error("depth image is empty");
  }
  if (depth_msg->data.empty())
  {
    throw std::runtime_error("depth image has no pixel data");
  }

  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(info_msg);
  if (!(camera_model.fx() > 0.0 && camera_model.fy() > 0.0))
  {
    throw std::runtime_error("camera intrinsics are invalid");
  }

  tf2::Quaternion body_to_world(
      imu_msg.orientation.x,
      imu_msg.orientation.y,
      imu_msg.orientation.z,
      imu_msg.orientation.w);
  if (!std::isfinite(imu_msg.orientation.x) || !std::isfinite(imu_msg.orientation.y) ||
      !std::isfinite(imu_msg.orientation.z) || !std::isfinite(imu_msg.orientation.w) ||
      body_to_world.length2() < 1e-8)
  {
    throw std::runtime_error("IMU orientation quaternion is invalid");
  }
  body_to_world.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(body_to_world).getRPY(roll, pitch, yaw);

  tf2::Quaternion remove_yaw;
  remove_yaw.setRPY(0.0, 0.0, -yaw);
  const tf2::Matrix3x3 level_from_body(remove_yaw * body_to_world);

  tf2::Quaternion mount_quaternion;
  mount_quaternion.setRPY(
      config_.camera_mount_roll,
      config_.camera_mount_pitch,
      config_.camera_mount_yaw);
  const tf2::Matrix3x3 mount_rotation(mount_quaternion);

  std::vector<GravityAlignedPoint> points;
  points.reserve((depth_msg->width / config_.depth_stride + 1) *
                 (depth_msg->height / config_.depth_stride + 1));
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    if (depth_msg->step < depth_msg->width * sizeof(uint16_t) ||
        depth_msg->data.size() < depth_msg->step * depth_msg->height)
    {
      throw std::runtime_error("16-bit depth image buffer is smaller than declared dimensions");
    }
    collectLevelPoints<uint16_t>(
        depth_msg, camera_model, mount_rotation, level_from_body,
        config_.depth_stride, config_.range_max * 1.5, &points);
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    if (depth_msg->step < depth_msg->width * sizeof(float) ||
        depth_msg->data.size() < depth_msg->step * depth_msg->height)
    {
      throw std::runtime_error("32-bit depth image buffer is smaller than declared dimensions");
    }
    collectLevelPoints<float>(
        depth_msg, camera_model, mount_rotation, level_from_body,
        config_.depth_stride, config_.range_max * 1.5, &points);
  }
  else
  {
    throw std::runtime_error("unsupported depth image encoding: " + depth_msg->encoding);
  }

  return convertLevelPoints(points, depth_msg->header, stats, clearing_scan);
}

double GravityAlignedDepthToLaserScan::signedDistance(
    const Plane& plane, const GravityAlignedPoint& point) const
{
  return plane.nx * point.x + plane.ny * point.y + plane.nz * point.z + plane.d;
}

bool GravityAlignedDepthToLaserScan::refineGroundPlane(
    const std::vector<GravityAlignedPoint>& candidates,
    const std::vector<std::size_t>& inlier_indices,
    Plane* plane) const
{
  if (!plane || inlier_indices.size() < 3)
  {
    return false;
  }

  double sx = 0.0;
  double sy = 0.0;
  double sz = 0.0;
  double sxx = 0.0;
  double syy = 0.0;
  double sxy = 0.0;
  double sxz = 0.0;
  double syz = 0.0;
  for (std::size_t i = 0; i < inlier_indices.size(); ++i)
  {
    const GravityAlignedPoint& point = candidates[inlier_indices[i]];
    sx += point.x;
    sy += point.y;
    sz += point.z;
    sxx += point.x * point.x;
    syy += point.y * point.y;
    sxy += point.x * point.y;
    sxz += point.x * point.z;
    syz += point.y * point.z;
  }
  const double count = static_cast<double>(inlier_indices.size());
  double system[3][4] = {
      {sxx, sxy, sx, sxz},
      {sxy, syy, sy, syz},
      {sx, sy, count, sz}};
  double coefficients[3] = {0.0, 0.0, 0.0};
  if (!solveThreeByThree(system, coefficients))
  {
    return false;
  }

  double nx = -coefficients[0];
  double ny = -coefficients[1];
  double nz = 1.0;
  double d = -coefficients[2];
  const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
  nx /= norm;
  ny /= norm;
  nz /= norm;
  d /= norm;
  const double tilt = std::acos(clampValue(nz, -1.0, 1.0));
  const double height = d / nz;
  if (tilt > config_.max_ground_tilt ||
      height < config_.min_camera_height || height > config_.max_camera_height)
  {
    return false;
  }

  plane->nx = nx;
  plane->ny = ny;
  plane->nz = nz;
  plane->d = d;
  plane->valid = true;
  return true;
}

GravityAlignedDepthToLaserScan::Plane GravityAlignedDepthToLaserScan::estimateGroundPlane(
    const std::vector<GravityAlignedPoint>& points,
    GravityAlignedFilterStats* stats) const
{
  std::vector<GravityAlignedPoint> candidates;
  candidates.reserve(std::min<std::size_t>(points.size(), config_.max_ransac_points));
  const double maximum_vertical_extent =
      config_.max_camera_height + config_.range_max * std::tan(config_.max_ground_tilt);
  const double minimum_z = -maximum_vertical_extent;
  const double maximum_z = -config_.min_camera_height +
                           config_.range_max * std::tan(config_.max_ground_tilt);

  std::vector<std::size_t> eligible_indices;
  eligible_indices.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    const GravityAlignedPoint& point = points[i];
    const double planar_range = std::hypot(point.x, point.y);
    if (point.x > 0.0 && planar_range >= config_.range_min &&
        planar_range <= config_.range_max && point.z >= minimum_z && point.z <= maximum_z)
    {
      eligible_indices.push_back(i);
    }
  }
  if (stats)
  {
    stats->ground_candidate_points = eligible_indices.size();
  }

  if (eligible_indices.size() > static_cast<std::size_t>(config_.max_ransac_points))
  {
    const double step = static_cast<double>(eligible_indices.size()) /
                        static_cast<double>(config_.max_ransac_points);
    for (int i = 0; i < config_.max_ransac_points; ++i)
    {
      candidates.push_back(points[eligible_indices[static_cast<std::size_t>(i * step)]]);
    }
  }
  else
  {
    for (std::size_t i = 0; i < eligible_indices.size(); ++i)
    {
      candidates.push_back(points[eligible_indices[i]]);
    }
  }

  Plane best_plane;
  std::vector<std::size_t> best_inliers;
  if (candidates.size() >= 3)
  {
    uint32_t random_state = 2166136261u ^ static_cast<uint32_t>(candidates.size());
    for (int iteration = 0; iteration < config_.ransac_iterations; ++iteration)
    {
      const std::size_t i1 = nextRandom(&random_state) % candidates.size();
      std::size_t i2 = nextRandom(&random_state) % candidates.size();
      std::size_t i3 = nextRandom(&random_state) % candidates.size();
      if (i1 == i2 || i1 == i3 || i2 == i3)
      {
        continue;
      }
      const GravityAlignedPoint& p1 = candidates[i1];
      const GravityAlignedPoint& p2 = candidates[i2];
      const GravityAlignedPoint& p3 = candidates[i3];
      const double ux = p2.x - p1.x;
      const double uy = p2.y - p1.y;
      const double uz = p2.z - p1.z;
      const double vx = p3.x - p1.x;
      const double vy = p3.y - p1.y;
      const double vz = p3.z - p1.z;
      double nx = uy * vz - uz * vy;
      double ny = uz * vx - ux * vz;
      double nz = ux * vy - uy * vx;
      const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (norm < 1e-8)
      {
        continue;
      }
      nx /= norm;
      ny /= norm;
      nz /= norm;
      if (nz < 0.0)
      {
        nx = -nx;
        ny = -ny;
        nz = -nz;
      }
      const double tilt = std::acos(clampValue(nz, -1.0, 1.0));
      if (tilt > config_.max_ground_tilt)
      {
        continue;
      }
      const double d = -(nx * p1.x + ny * p1.y + nz * p1.z);
      const double height = d / nz;
      if (height < config_.min_camera_height || height > config_.max_camera_height)
      {
        continue;
      }

      Plane candidate_plane;
      candidate_plane.nx = nx;
      candidate_plane.ny = ny;
      candidate_plane.nz = nz;
      candidate_plane.d = d;
      candidate_plane.valid = true;
      std::vector<std::size_t> inliers;
      for (std::size_t index = 0; index < candidates.size(); ++index)
      {
        if (std::fabs(signedDistance(candidate_plane, candidates[index])) <=
            config_.ground_distance_threshold)
        {
          inliers.push_back(index);
        }
      }
      if (inliers.size() > best_inliers.size())
      {
        best_plane = candidate_plane;
        best_inliers.swap(inliers);
      }
    }
  }

  const std::size_t required_by_ratio = static_cast<std::size_t>(
      std::ceil(config_.min_ground_inlier_ratio * static_cast<double>(candidates.size())));
  const std::size_t required_inliers = std::max<std::size_t>(
      static_cast<std::size_t>(config_.min_ground_inliers), required_by_ratio);
  if (best_plane.valid && best_inliers.size() >= required_inliers &&
      refineGroundPlane(candidates, best_inliers, &best_plane))
  {
    std::size_t refined_inliers = 0;
    for (std::size_t index = 0; index < candidates.size(); ++index)
    {
      if (std::fabs(signedDistance(best_plane, candidates[index])) <=
          config_.ground_distance_threshold)
      {
        ++refined_inliers;
      }
    }
    if (refined_inliers >= required_inliers)
    {
      if (stats)
      {
        stats->ground_plane_detected = true;
        stats->ground_inlier_points = refined_inliers;
        stats->ground_height = best_plane.d / best_plane.nz;
        stats->ground_tilt = std::acos(clampValue(best_plane.nz, -1.0, 1.0));
      }
      return best_plane;
    }
  }

  Plane fallback;
  if (config_.fallback_to_nominal_ground)
  {
    fallback.nx = 0.0;
    fallback.ny = 0.0;
    fallback.nz = 1.0;
    fallback.d = config_.nominal_camera_height;
    fallback.valid = true;
    if (stats)
    {
      stats->used_nominal_ground = true;
      stats->ground_height = config_.nominal_camera_height;
      stats->ground_tilt = 0.0;
    }
  }
  return fallback;
}

sensor_msgs::LaserScanPtr GravityAlignedDepthToLaserScan::makeScan(
    const std_msgs::Header& header) const
{
  sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan());
  scan->header = header;
  scan->header.frame_id = config_.output_frame_id;
  scan->angle_min = config_.angle_min;
  scan->angle_increment = config_.angle_increment;
  const std::size_t count = static_cast<std::size_t>(
      std::floor((config_.angle_max - config_.angle_min) / config_.angle_increment)) + 1;
  scan->angle_max = config_.angle_min + (count - 1) * config_.angle_increment;
  scan->time_increment = 0.0;
  scan->scan_time = config_.scan_time;
  scan->range_min = config_.range_min;
  scan->range_max = config_.range_max;
  scan->ranges.assign(count, std::numeric_limits<float>::quiet_NaN());
  return scan;
}

sensor_msgs::LaserScanPtr GravityAlignedDepthToLaserScan::convertLevelPoints(
    const std::vector<GravityAlignedPoint>& points,
    const std_msgs::Header& header,
    GravityAlignedFilterStats* stats,
    sensor_msgs::LaserScanPtr* clearing_scan) const
{
  validateConfig();
  GravityAlignedFilterStats local_stats;
  local_stats.valid_depth_points = points.size();
  const Plane ground_plane = estimateGroundPlane(points, &local_stats);
  sensor_msgs::LaserScanPtr scan = makeScan(header);
  sensor_msgs::LaserScanPtr clear_scan = makeScan(header);
  if (!ground_plane.valid)
  {
    if (stats)
    {
      *stats = local_stats;
    }
    if (clearing_scan)
    {
      *clearing_scan = clear_scan;
    }
    return scan;
  }

  std::vector<bool> observed(scan->ranges.size(), false);
  std::vector<bool> occupied(scan->ranges.size(), false);
  std::vector<float> farthest_ground_range(
      scan->ranges.size(), std::numeric_limits<float>::quiet_NaN());
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    const GravityAlignedPoint& point = points[i];
    const double planar_range = std::hypot(point.x, point.y);
    if (!std::isfinite(planar_range) || planar_range < config_.range_min ||
        planar_range > config_.range_max || point.x <= 0.0)
    {
      continue;
    }
    const double angle = std::atan2(point.y, point.x);
    if (angle < scan->angle_min || angle > scan->angle_max)
    {
      continue;
    }
    std::size_t index = static_cast<std::size_t>(
        std::floor((angle - scan->angle_min) / scan->angle_increment));
    if (index >= scan->ranges.size())
    {
      index = scan->ranges.size() - 1;
    }
    observed[index] = true;

    const double height = signedDistance(ground_plane, point);
    if (height < config_.min_obstacle_height)
    {
      // Only points on/just above the fitted plane prove traversable ground.
      // Reject below-plane depth outliers instead of letting them extend a
      // clearing ray.
      if (height >= -config_.ground_distance_threshold &&
          (!std::isfinite(farthest_ground_range[index]) ||
           planar_range > farthest_ground_range[index]))
      {
        farthest_ground_range[index] = static_cast<float>(planar_range);
      }
      continue;
    }
    if (height > config_.max_obstacle_height)
    {
      continue;
    }
    if (!occupied[index] || planar_range < scan->ranges[index])
    {
      scan->ranges[index] = static_cast<float>(planar_range);
      occupied[index] = true;
    }
    ++local_stats.obstacle_points;
  }

  for (std::size_t index = 0; index < observed.size(); ++index)
  {
    if (observed[index])
    {
      ++local_stats.observed_scan_bins;
      // Never clear through a currently detected obstacle.  If no obstacle is
      // present, the farthest actually observed ground return bounds known
      // free space without inventing an infinite/no-return measurement.
      if (occupied[index])
      {
        clear_scan->ranges[index] = scan->ranges[index];
      }
      else if (std::isfinite(farthest_ground_range[index]))
      {
        clear_scan->ranges[index] = farthest_ground_range[index];
      }
      if (std::isfinite(clear_scan->ranges[index]))
      {
        ++local_stats.clearing_scan_bins;
      }
      if (config_.use_inf && !occupied[index])
      {
        scan->ranges[index] = std::numeric_limits<float>::infinity();
      }
    }
  }
  if (stats)
  {
    *stats = local_stats;
  }
  if (clearing_scan)
  {
    *clearing_scan = clear_scan;
  }
  return scan;
}

}  // namespace depthimage_to_laserscan
