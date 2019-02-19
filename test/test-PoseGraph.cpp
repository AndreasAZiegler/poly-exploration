#include "PoseGraph.h"
#include "gtest/gtest.h"

namespace {

// The fixture for testing class Polygon.
class PoseGraphTest : public ::testing::Test {
 protected:
  PoseGraphTest() = default;
};

// Tests that PoseGraph is constructed correctly
TEST_F(PoseGraphTest, CreateOnePoseGraphPose) {
  PoseGraph pose_graph;

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

  pose_graph.addPose(polygon, Pose());

  auto pose_graph_pose = pose_graph.getPoseGraphPose();
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

TEST_F(PoseGraphTest, CreateTwoPoseGraphPoses) {
  PoseGraph pose_graph;

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

  pose_graph.addPose(polygon, Pose());

  auto pose_graph_pose_1 = pose_graph.getPoseGraphPose();
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

  pose_graph.addPose(polygon, Pose(Position(0.5, 0.5, 0), Rotation()));

  auto pose_graph_pose_2 = pose_graph.getPoseGraphPose();
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
