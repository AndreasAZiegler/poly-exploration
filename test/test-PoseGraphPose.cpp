#include <memory>
#include <vector>
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

  auto adjacent_poses = pose_graph_pose.getAdjacentPoses();
  ASSERT_EQ(adjacent_poses.empty(), true)
      << "There should not be any adjacent pose";
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

  auto adjacent_poses_1 = pose_graph_pose_1.getAdjacentPoses();
  ASSERT_EQ(adjacent_poses_1.empty(), true)
      << "There should not be any adjacent pose";

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

  auto adjacent_poses_2 = pose_graph_pose_1.getAdjacentPoses();
  ASSERT_EQ(adjacent_poses_2.empty(), true)
      << "There should not be any adjacent pose";

  Pose transformation(Position(0.5, 0.5, 0), Rotation());

  pose_graph_pose_1.addAdjacentPose(1, transformation);

  adjacent_poses_1 = pose_graph_pose_1.getAdjacentPoses();
  ASSERT_EQ(adjacent_poses_1.size(), 1) << "There should be one adjacent pose";

  unsigned int id_pose_graph_pose_2 = 1;
  Pose return_transformation_1 = adjacent_poses_1[id_pose_graph_pose_2];
  ASSERT_EQ(return_transformation_1, transformation)
      << "There should be one adjacent pose";

  auto inverse_rotation = transformation.getRotation().inverted();
  Pose inverted_transformation(
      -inverse_rotation.rotate(transformation.getPosition()), inverse_rotation);

  pose_graph_pose_2.addAdjacentPose(0, inverted_transformation);

  adjacent_poses_2 = pose_graph_pose_2.getAdjacentPoses();
  ASSERT_EQ(adjacent_poses_2.size(), 1) << "There should be one adjacent pose";

  unsigned int id_pose_graph_pose_1 = 0;
  Pose return_transformation_2 = adjacent_poses_2[id_pose_graph_pose_1];
  ASSERT_EQ(return_transformation_2, inverted_transformation)
      << "There should be one adjacent pose";
}

}  // namespace
