#include <depthimage_to_laserscan/GravityAlignedDepthToLaserScan.h>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace depthimage_to_laserscan
{
namespace
{

GravityAlignedFilterConfig testConfig()
{
  GravityAlignedFilterConfig config;
  config.range_min = 0.1;
  config.range_max = 3.0;
  config.angle_min = -0.75;
  config.angle_max = 0.75;
  config.angle_increment = 0.01;
  config.min_camera_height = 0.2;
  config.max_camera_height = 0.8;
  config.nominal_camera_height = 0.42;
  config.ground_distance_threshold = 0.025;
  config.min_obstacle_height = 0.06;
  config.max_obstacle_height = 1.5;
  config.min_ground_inliers = 100;
  config.min_ground_inlier_ratio = 0.1;
  config.ransac_iterations = 80;
  config.max_ransac_points = 6000;
  config.fallback_to_nominal_ground = false;
  return config;
}

std::vector<GravityAlignedPoint> makeGroundAndWall()
{
  std::vector<GravityAlignedPoint> points;
  // A slightly sloped floor: z = 0.02*x - 0.01*y - 0.42.
  for (int ix = 0; ix < 70; ++ix)
  {
    const double x = 0.25 + 0.035 * ix;
    for (int iy = -35; iy <= 35; ++iy)
    {
      const double y = 0.02 * iy;
      points.push_back(GravityAlignedPoint(x, y, 0.02 * x - 0.01 * y - 0.42));
    }
  }

  // A vertical obstacle centered in front of the camera.
  for (int iy = -20; iy <= 20; ++iy)
  {
    const double y = 0.005 * iy;
    for (int iz = 0; iz < 30; ++iz)
    {
      const double ground_z = 0.02 * 2.0 - 0.01 * y - 0.42;
      points.push_back(GravityAlignedPoint(2.0, y, ground_z + 0.08 + 0.02 * iz));
    }
  }
  return points;
}

TEST(GravityAlignedDepthToLaserScan, RemovesFloorAndKeepsVerticalObstacle)
{
  GravityAlignedDepthToLaserScan converter;
  converter.setConfig(testConfig());
  GravityAlignedFilterStats stats;
  std_msgs::Header header;
  header.frame_id = "camera";
  const sensor_msgs::LaserScanPtr scan =
      converter.convertLevelPoints(makeGroundAndWall(), header, &stats);

  ASSERT_TRUE(stats.ground_plane_detected);
  EXPECT_FALSE(stats.used_nominal_ground);
  EXPECT_NEAR(0.42, stats.ground_height, 0.015);
  EXPECT_LT(stats.ground_tilt, 0.04);
  EXPECT_GT(stats.ground_inlier_points, 1000u);
  EXPECT_GT(stats.obstacle_points, 100u);
  EXPECT_EQ("scan_ground_filtered_link", scan->header.frame_id);

  const std::size_t center = static_cast<std::size_t>(
      std::floor((0.0 - scan->angle_min) / scan->angle_increment));
  ASSERT_LT(center, scan->ranges.size());
  EXPECT_NEAR(2.0, scan->ranges[center], 0.02);

  // Most floor-only rays stay invalid instead of becoming false obstacles.
  std::size_t finite_bins = 0;
  for (std::size_t i = 0; i < scan->ranges.size(); ++i)
  {
    if (std::isfinite(scan->ranges[i]))
    {
      ++finite_bins;
    }
  }
  EXPECT_LT(finite_bins, 20u);
}

TEST(GravityAlignedDepthToLaserScan, ProducesEmptyScanWhenNoGroundAndFallbackDisabled)
{
  GravityAlignedDepthToLaserScan converter;
  GravityAlignedFilterConfig config = testConfig();
  converter.setConfig(config);
  std::vector<GravityAlignedPoint> points;
  for (int i = 0; i < 100; ++i)
  {
    points.push_back(GravityAlignedPoint(1.0, 0.001 * i, 0.30));
  }

  GravityAlignedFilterStats stats;
  const sensor_msgs::LaserScanPtr scan =
      converter.convertLevelPoints(points, std_msgs::Header(), &stats);
  EXPECT_FALSE(stats.ground_plane_detected);
  EXPECT_FALSE(stats.used_nominal_ground);
  for (std::size_t i = 0; i < scan->ranges.size(); ++i)
  {
    EXPECT_TRUE(std::isnan(scan->ranges[i]));
  }
}

}  // namespace
}  // namespace depthimage_to_laserscan
