#include "PoseGraphPose.h"
#include "gtest/gtest.h"

namespace {

// The fixture for testing class Polygon.
class PoseGraphPoseTest : public ::testing::Test {
 protected:
  PoseGraphPoseTest() = default;
};

// Tests that PoseGraphPose is constructed correctly
TEST_F(PoseGraphPoseTest, CreateOnePoseGraphPose) {
  std::vector<PolygonPoint> points;
  points.emplace_back(0.0, 0.0, false);
  points.emplace_back(10.0, 10.0, false);
  points.emplace_back(10.0, 7.5, false);
  points.emplace_back(10.0, 5.0, false);
  points.emplace_back(10.0, 2.5, false);
  points.emplace_back(10.0, 0.0, false);
  points.emplace_back(10.0, -2.5, false);
  points.emplace_back(10.0, -5.0, false);
  points.emplace_back(10.0, -7.5, false);
  points.emplace_back(10.0, -10.0, false);
  points.emplace_back(0.0, 0.0, false);

  std::vector<bool> maximum_ranges{false, false, false, false, false, false,
                                   false, false, false, false, false};

  Polygon polygon(points, maximum_ranges);

  PoseGraphPose pose_graph_pose(polygon, 0);

  auto return_points = pose_graph_pose.getPolygon().getPoints();

  ASSERT_EQ(points.size(), return_points.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_points.size(); ++i) {
    EXPECT_EQ(points[i], return_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  ASSERT_EQ(polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";

  ASSERT_EQ(pose_graph_pose.getId(), 0) << "Wrong pose graph pose id";
}

TEST_F(PoseGraphPoseTest, CreateTwoPoseGraphPose) {
  std::vector<PolygonPoint> points;
  points.emplace_back(0.0, 0.0, false);
  points.emplace_back(10.0, 10.0, false);
  points.emplace_back(10.0, 7.5, false);
  points.emplace_back(10.0, 5.0, false);
  points.emplace_back(10.0, 2.5, false);
  points.emplace_back(10.0, 0.0, false);
  points.emplace_back(10.0, -2.5, false);
  points.emplace_back(10.0, -5.0, false);
  points.emplace_back(10.0, -7.5, false);
  points.emplace_back(10.0, -10.0, false);
  points.emplace_back(0.0, 0.0, false);

  std::vector<bool> maximum_ranges{false, false, false, false, false, false,
                                   false, false, false, false, false};

  Polygon polygon(points, maximum_ranges);

  PoseGraphPose pose_graph_pose_1(polygon, 0);

  auto return_points_1 = pose_graph_pose_1.getPolygon().getPoints();

  ASSERT_EQ(points.size(), return_points_1.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_points_1.size(); ++i) {
    EXPECT_EQ(points[i], return_points_1[i])
        << "Vectors points and return_points differ at index " << i;
  }

  ASSERT_EQ(polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";

  ASSERT_EQ(pose_graph_pose_1.getId(), 0) << "Wrong pose graph pose id";

  PoseGraphPose pose_graph_pose_2(polygon, 1);

  auto return_points_2 = pose_graph_pose_2.getPolygon().getPoints();

  ASSERT_EQ(points.size(), return_points_2.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_points_2.size(); ++i) {
    EXPECT_EQ(points[i], return_points_2[i])
        << "Vectors points and return_points differ at index " << i;
  }

  ASSERT_EQ(polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";

  ASSERT_EQ(pose_graph_pose_2.getId(), 1) << "Wrong pose graph pose id";
}

}  // namespace
