/*  PolygonConsolidation unit test
 */

#include "PolygonConsolidation.h"
#include "gtest/gtest.h"

// The fixture for testing class Polygon.
class PolygonConsolidationTest : public ::testing::Test {
 protected:
  PolygonConsolidationTest() = default;
};

TEST_F(PolygonConsolidationTest, addAdjacentCandidates) {
  PoseGraph pose_graph;

  std::vector<PolygonPoint> points;
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 10.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 7.5, PointType::OBSTACLE);
  points.emplace_back(10.0, 5.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 2.5, PointType::OBSTACLE);
  points.emplace_back(10.0, 0.0, PointType::OBSTACLE);
  points.emplace_back(10.0, -2.5, PointType::OBSTACLE);
  points.emplace_back(10.0, -5.0, PointType::OBSTACLE);
  points.emplace_back(10.0, -7.5, PointType::OBSTACLE);
  points.emplace_back(10.0, -10.0, PointType::OBSTACLE);
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon polygon(points);

  pose_graph.addPose(polygon, Pose());

  Pose transformation(Position(5.0, 0.0, 0), Rotation());

  pose_graph.addPose(polygon, transformation);

  Pose identity_transformation;
  std::stack<std::tuple<unsigned int, unsigned int, Pose>> candidates;
  std::set<unsigned int> checked_candidates_id;

  unsigned int pose_graph_pose_id = 0;
  auto pose_graph_poses = pose_graph.getPoseGraphPoses();
  PolygonConsolidation::addAdjacentCandidates(
      pose_graph_pose_id, identity_transformation, candidates,
      checked_candidates_id, pose_graph_poses);
  
  EXPECT_EQ(checked_candidates_id.count(1), 1) << "Wrong checked candidates.";

  auto candidate_id = std::get<0>(candidates.top());
  EXPECT_EQ(candidate_id, 1) << "Wrong candidate id";

  auto previous_candidate_id = std::get<1>(candidates.top());
  EXPECT_EQ(previous_candidate_id, 0) << "Wrong previous candidate id";

  Pose expected_transformation(Position(-5.0, 0.0, 0), Rotation());
  auto candidate_transformation = std::get<2>(candidates.top());
  EXPECT_EQ(candidate_transformation, expected_transformation) << "Wrong transformation";

  candidates.pop();
  checked_candidates_id.clear();
  pose_graph_pose_id = 1;
  PolygonConsolidation::addAdjacentCandidates(
      pose_graph_pose_id, identity_transformation, candidates,
      checked_candidates_id, pose_graph_poses);
  
  EXPECT_EQ(checked_candidates_id.count(0), 1) << "Wrong checked candidates.";

  candidate_id = std::get<0>(candidates.top());
  EXPECT_EQ(candidate_id, 0) << "Wrong candidate id";

  previous_candidate_id = std::get<1>(candidates.top());
  EXPECT_EQ(previous_candidate_id, 1) << "Wrong previous candidate id";

  expected_transformation = Pose(Position(5.0, 0.0, 0), Rotation());
  candidate_transformation = std::get<2>(candidates.top());
  EXPECT_EQ(candidate_transformation, expected_transformation) << "Wrong transformation";
}

TEST_F(PolygonConsolidationTest, GetIntersectedPolygonOwners1) {
  PoseGraph pose_graph;

  std::vector<PolygonPoint> points;
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 10.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 7.5, PointType::OBSTACLE);
  points.emplace_back(10.0, 5.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 2.5, PointType::OBSTACLE);
  points.emplace_back(10.0, 0.0, PointType::OBSTACLE);
  points.emplace_back(10.0, -2.5, PointType::OBSTACLE);
  points.emplace_back(10.0, -5.0, PointType::OBSTACLE);
  points.emplace_back(10.0, -7.5, PointType::OBSTACLE);
  points.emplace_back(10.0, -10.0, PointType::OBSTACLE);
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon polygon(points);

  pose_graph.addPose(polygon, Pose());

  Pose transformation(Position(5.0, 0.0, 0), Rotation());

  pose_graph.addPose(polygon, transformation);

  unsigned int result_id = 0;
  Pose result_transformation = Pose(Position(5.0, 0.0, 0.0), Rotation());

  std::vector<PoseGraphPose> pose_graph_poses = pose_graph.getPoseGraphPoses();

  auto intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(1, pose_graph_poses);
  auto test_id = std::get<0>(intersected_polygon_owners.front());
  auto test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, result_transformation);

  intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(0, pose_graph_poses);
  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  result_id = 1;
  result_transformation = Pose(Position(-5.0, 0.0, 0.0), Rotation());
  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, result_transformation);
}
