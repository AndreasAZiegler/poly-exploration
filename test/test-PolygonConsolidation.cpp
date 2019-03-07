/*  PolygonConsolidation unit test
 */

#include <vector>
#include <set>
#include "PolygonConsolidation.h"
#include "gtest/gtest.h"

// The fixture for testing class Polygon.
class PolygonConsolidationTest : public ::testing::Test {
 protected:
  PolygonConsolidationTest() = default;
};

TEST_F(PolygonConsolidationTest, GetAdjacentCandidates1) {
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
  std::set<unsigned int> checked_candidates_id;

  unsigned int pose_graph_pose_id = 0;
  auto pose_graph_poses = pose_graph.getPoseGraphPoses();
  auto candidates = PolygonConsolidation::addAdjacentCandidates(
      pose_graph_pose_id, identity_transformation, checked_candidates_id,
      pose_graph_poses);

  EXPECT_EQ(checked_candidates_id.count(1), 1) << "Wrong checked candidates.";

  auto candidate_id = std::get<0>(candidates.at(0));
  EXPECT_EQ(candidate_id, 1) << "Wrong candidate id";

  auto previous_candidate_id = std::get<1>(candidates.at(0));
  EXPECT_EQ(previous_candidate_id, 0) << "Wrong previous candidate id";

  Pose expected_transformation(Position(-5.0, 0.0, 0), Rotation());
  auto candidate_transformation = std::get<2>(candidates.at(0));
  EXPECT_EQ(candidate_transformation, expected_transformation)
      << "Wrong transformation";

  checked_candidates_id.clear();
  pose_graph_pose_id = 1;
  candidates = PolygonConsolidation::addAdjacentCandidates(
      pose_graph_pose_id, identity_transformation, checked_candidates_id,
      pose_graph_poses);

  EXPECT_EQ(checked_candidates_id.count(0), 1) << "Wrong checked candidates.";

  candidate_id = std::get<0>(candidates.at(0));
  EXPECT_EQ(candidate_id, 0) << "Wrong candidate id";

  previous_candidate_id = std::get<1>(candidates.at(0));
  EXPECT_EQ(previous_candidate_id, 1) << "Wrong previous candidate id";

  expected_transformation = Pose(Position(5.0, 0.0, 0), Rotation());
  candidate_transformation = std::get<2>(candidates.at(0));
  EXPECT_EQ(candidate_transformation, expected_transformation)
      << "Wrong transformation";
}

TEST_F(PolygonConsolidationTest, GetAdjacentCandidates2) {
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

  kindr::AngleAxisD angleAxis1(0.7853 /*45deg*/, Eigen::Vector3d::UnitZ());
  Pose transformation1(Position(5.0, 0.0, 0), Rotation(angleAxis1));

  pose_graph.addPose(polygon, transformation1);

  kindr::AngleAxisD angleAxis2(-0.7853 /*45deg*/, Eigen::Vector3d::UnitZ());
  Pose transformation2(Position(5.0, 0.0, 0), Rotation(angleAxis2));

  pose_graph.addPose(polygon, transformation2);

  auto pose_graph_poses = pose_graph.getPoseGraphPoses();

  /*
  auto intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(0, pose_graph_poses);
  */
  Pose identity_transformation;
  std::set<unsigned int> checked_candidates_id;

  unsigned int pose_graph_pose_id = 0;
  pose_graph_poses = pose_graph.getPoseGraphPoses();
  auto candidates = PolygonConsolidation::addAdjacentCandidates(
      pose_graph_pose_id, identity_transformation, checked_candidates_id,
      pose_graph_poses);

  EXPECT_EQ(candidates.size(), 1) << "Wrong number of adjacent poses!";

  EXPECT_EQ(checked_candidates_id.count(1), 1) << "Wrong checked candidates.";

  auto candidate_id = std::get<0>(candidates.at(0));
  EXPECT_EQ(candidate_id, 1) << "Wrong candidate id";

  auto previous_candidate_id = std::get<1>(candidates.at(0));
  EXPECT_EQ(previous_candidate_id, 0) << "Wrong previous candidate id";

  auto inverse_rotation = transformation1.getRotation().inverted();
  Pose expected_transformation(
      -inverse_rotation.rotate(transformation1.getPosition()),
      inverse_rotation);
  auto candidate_transformation = std::get<2>(candidates.at(0));
  EXPECT_EQ(candidate_transformation, expected_transformation)
      << "Wrong transformation";

  checked_candidates_id.clear();
  pose_graph_pose_id = 1;
  candidates = PolygonConsolidation::addAdjacentCandidates(
      pose_graph_pose_id, identity_transformation, checked_candidates_id,
      pose_graph_poses);

  EXPECT_EQ(candidates.size(), 2) << "Wrong number of adjacent poses!";

  EXPECT_EQ(checked_candidates_id.count(2), 1) << "Wrong checked candidates.";

  candidate_id = std::get<0>(candidates.at(1));
  EXPECT_EQ(candidate_id, 2) << "Wrong candidate id";

  previous_candidate_id = std::get<1>(candidates.at(1));
  EXPECT_EQ(previous_candidate_id, 1) << "Wrong previous candidate id";

  expected_transformation = Pose(Position(5.0, 0.0, 0), Rotation());

  inverse_rotation = transformation2.getRotation().inverted();
  expected_transformation =
      Pose(-inverse_rotation.rotate(transformation2.getPosition()),
           inverse_rotation);
  candidate_transformation = std::get<2>(candidates.at(1));
  EXPECT_EQ(candidate_transformation, expected_transformation)
      << "Wrong transformation";

  EXPECT_EQ(checked_candidates_id.count(0), 1) << "Wrong checked candidates.";

  candidate_id = std::get<0>(candidates.at(0));
  EXPECT_EQ(candidate_id, 0) << "Wrong candidate id";

  previous_candidate_id = std::get<1>(candidates.at(0));
  EXPECT_EQ(previous_candidate_id, 1) << "Wrong previous candidate id";

  expected_transformation = transformation1;
  candidate_transformation = std::get<2>(candidates.at(0));
  EXPECT_EQ(candidate_transformation, expected_transformation)
      << "Wrong transformation";

  checked_candidates_id.clear();
  pose_graph_pose_id = 2;
  candidates = PolygonConsolidation::addAdjacentCandidates(
      pose_graph_pose_id, identity_transformation, checked_candidates_id,
      pose_graph_poses);

  EXPECT_EQ(candidates.size(), 1) << "Wrong number of adjacent poses!";

  EXPECT_EQ(checked_candidates_id.count(1), 1) << "Wrong checked candidates.";

  candidate_id = std::get<0>(candidates.at(0));
  EXPECT_EQ(candidate_id, 1) << "Wrong candidate id";

  previous_candidate_id = std::get<1>(candidates.at(0));
  EXPECT_EQ(previous_candidate_id, 2) << "Wrong previous candidate id";

  expected_transformation = transformation2;
  candidate_transformation = std::get<2>(candidates.at(0));
  EXPECT_EQ(candidate_transformation, expected_transformation)
      << "Wrong transformation";
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

TEST_F(PolygonConsolidationTest, GetIntersectedPolygonOwners2) {
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

  kindr::AngleAxisD angleAxis1(0 /*0.7853 45deg*/, Eigen::Vector3d::UnitZ());
  Pose transformation1(Position(5.0, 5.0, 0), Rotation(angleAxis1));

  pose_graph.addPose(polygon, transformation1);

  kindr::AngleAxisD angleAxis2(0 /*-0.7853 -45deg*/, Eigen::Vector3d::UnitZ());
  Pose transformation2(Position(5.0, -5.0, 0), Rotation(angleAxis2));

  pose_graph.addPose(polygon, transformation2);

  std::vector<PoseGraphPose> pose_graph_poses = pose_graph.getPoseGraphPoses();

  auto intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(0, pose_graph_poses);

  EXPECT_EQ(intersected_polygon_owners.size(), 2)
      << "Wrong number of intersecting polygons.";

  unsigned int result_id = 1;

  auto inverse_rotation = transformation1.getRotation().inverted();
  Pose result_transformation(
      -inverse_rotation.rotate(transformation1.getPosition()),
      inverse_rotation);

  auto test_id = std::get<0>(intersected_polygon_owners.front());
  auto test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, result_transformation);

  intersected_polygon_owners.pop();

  EXPECT_EQ(intersected_polygon_owners.size(), 1)
      << "Wrong number of intersecting polygons.";

  result_id = 2;

  Pose stacked_transformation = transformation2 * transformation1;
  inverse_rotation = stacked_transformation.getRotation().inverted();
  result_transformation =
      Pose(-inverse_rotation.rotate(stacked_transformation.getPosition()),
           inverse_rotation);

  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  // EXPECT_EQ(test_transformation, result_transformation);
  EXPECT_NEAR(test_transformation.getPosition().x(),
              result_transformation.getPosition().x(), 1.0e-6)
      << "Wrong transformation!";
  EXPECT_NEAR(test_transformation.getPosition().y(),
              result_transformation.getPosition().y(), 1.0e-6)
      << "Wrong transformation!";
  EXPECT_NEAR(test_transformation.getPosition().z(),
              result_transformation.getPosition().z(), 1.0e-6)
      << "Wrong transformation!";
  ASSERT_TRUE(test_transformation.getRotation().isNear(
      result_transformation.getRotation(), 1.0e-6))
      << "Wrong transformation!";

  intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(1, pose_graph_poses);

  EXPECT_EQ(intersected_polygon_owners.size(), 2)
      << "Wrong number of intersecting polygons.";

  result_id = 2;

  inverse_rotation = transformation2.getRotation().inverted();
  result_transformation =
      Pose(-inverse_rotation.rotate(transformation2.getPosition()),
           inverse_rotation);

  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, result_transformation);

  intersected_polygon_owners.pop();

  EXPECT_EQ(intersected_polygon_owners.size(), 1)
      << "Wrong number of intersecting polygons.";

  result_id = 0;

  result_transformation = transformation1;

  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, result_transformation);

  intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(2, pose_graph_poses);

  EXPECT_EQ(intersected_polygon_owners.size(), 2)
      << "Wrong number of intersecting polygons.";

  result_id = 1;

  result_transformation = transformation2;

  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, result_transformation);

  intersected_polygon_owners.pop();

  EXPECT_EQ(intersected_polygon_owners.size(), 1)
      << "Wrong number of intersecting polygons.";

  result_id = 0;

  result_transformation = transformation1 * transformation2;

  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, result_transformation);
}
