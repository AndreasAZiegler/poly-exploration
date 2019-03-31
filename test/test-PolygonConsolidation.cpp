/*  PolygonConsolidation unit test
 */

#include <set>
#include <vector>
#include "poly_exploration/PolygonConsolidation.h"
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

  pose_graph.addPose(Pose(), polygon);

  Pose transformation(Position(5.0, 0.0, 0), Rotation());

  pose_graph.addPose(transformation, polygon);

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

  auto expected_transformation = transformation;
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

  expected_transformation = Pose(Position(-5.0, 0.0, 0), Rotation());
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

  pose_graph.addPose(Pose(), polygon);

  kindr::AngleAxisD angleAxis1(0.7853 /*45deg*/, Eigen::Vector3d::UnitZ());
  Pose transformation1(Position(5.0, 0.0, 0), Rotation(angleAxis1));

  pose_graph.addPose(transformation1, polygon);

  kindr::AngleAxisD angleAxis2(-0.7853 /*45deg*/, Eigen::Vector3d::UnitZ());
  Pose transformation2(Position(5.0, 0.0, 0), Rotation(angleAxis2));

  pose_graph.addPose(transformation2, polygon);

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

  auto expected_transformation = transformation1;
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

  expected_transformation = transformation2;
  candidate_transformation = std::get<2>(candidates.at(1));
  EXPECT_EQ(candidate_transformation, expected_transformation)
      << "Wrong transformation";

  EXPECT_EQ(checked_candidates_id.count(0), 1) << "Wrong checked candidates.";

  candidate_id = std::get<0>(candidates.at(0));
  EXPECT_EQ(candidate_id, 0) << "Wrong candidate id";

  previous_candidate_id = std::get<1>(candidates.at(0));
  EXPECT_EQ(previous_candidate_id, 1) << "Wrong previous candidate id";

  expected_transformation = transformation1;

  auto inverse_rotation = transformation1.getRotation().inverted();
  expected_transformation =
      Pose(-inverse_rotation.rotate(transformation1.getPosition()),
           inverse_rotation);

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

  inverse_rotation = transformation2.getRotation().inverted();
  expected_transformation =
      Pose(-inverse_rotation.rotate(transformation2.getPosition()),
           inverse_rotation);

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

  pose_graph.addPose(Pose(), polygon);

  Pose transformation(Position(5.0, 0.0, 0), Rotation());

  pose_graph.addPose(transformation, polygon);

  unsigned int result_id = 0;
  Pose expected_transformation = Pose(Position(-5.0, 0.0, 0), Rotation());

  std::vector<PoseGraphPose> pose_graph_poses = pose_graph.getPoseGraphPoses();

  auto intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(1, pose_graph_poses);
  auto test_id = std::get<0>(intersected_polygon_owners.front());
  auto test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, expected_transformation);

  intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(0, pose_graph_poses);
  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  result_id = 1;
  expected_transformation = transformation;
  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, expected_transformation);
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

  pose_graph.addPose(Pose(), polygon);

  kindr::AngleAxisD angleAxis1(0 /*0.7853 45deg*/, Eigen::Vector3d::UnitZ());
  Pose transformation1(Position(5.0, 5.0, 0), Rotation(angleAxis1));

  pose_graph.addPose(transformation1, polygon);

  kindr::AngleAxisD angleAxis2(0 /*-0.7853 -45deg*/, Eigen::Vector3d::UnitZ());
  Pose transformation2(Position(5.0, -5.0, 0), Rotation(angleAxis2));

  pose_graph.addPose(transformation2, polygon);

  std::vector<PoseGraphPose> pose_graph_poses = pose_graph.getPoseGraphPoses();

  auto intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(0, pose_graph_poses);

  EXPECT_EQ(intersected_polygon_owners.size(), 2)
      << "Wrong number of intersecting polygons.";

  unsigned int result_id = 1;

  Pose expected_transformation = transformation1;

  auto test_id = std::get<0>(intersected_polygon_owners.front());
  auto test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, expected_transformation);

  intersected_polygon_owners.pop();

  EXPECT_EQ(intersected_polygon_owners.size(), 1)
      << "Wrong number of intersecting polygons.";

  result_id = 2;

  expected_transformation = transformation2 * transformation1;

  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_NEAR(test_transformation.getPosition().x(),
              expected_transformation.getPosition().x(), 1.0e-6)
      << "Wrong transformation!";
  EXPECT_NEAR(test_transformation.getPosition().y(),
              expected_transformation.getPosition().y(), 1.0e-6)
      << "Wrong transformation!";
  EXPECT_NEAR(test_transformation.getPosition().z(),
              expected_transformation.getPosition().z(), 1.0e-6)
      << "Wrong transformation!";
  ASSERT_TRUE(test_transformation.getRotation().isNear(
      expected_transformation.getRotation(), 1.0e-6))
      << "Wrong transformation!";

  intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(1, pose_graph_poses);

  EXPECT_EQ(intersected_polygon_owners.size(), 2)
      << "Wrong number of intersecting polygons.";

  result_id = 2;

  expected_transformation = transformation2;

  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, expected_transformation);

  intersected_polygon_owners.pop();

  EXPECT_EQ(intersected_polygon_owners.size(), 1)
      << "Wrong number of intersecting polygons.";

  result_id = 0;

  auto inverse_rotation = transformation1.getRotation().inverted();
  expected_transformation =
      Pose(-inverse_rotation.rotate(transformation1.getPosition()),
           inverse_rotation);

  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, expected_transformation);

  intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(2, pose_graph_poses);

  EXPECT_EQ(intersected_polygon_owners.size(), 2)
      << "Wrong number of intersecting polygons.";

  result_id = 1;

  expected_transformation = transformation2;
  inverse_rotation = transformation2.getRotation().inverted();
  expected_transformation =
      Pose(-inverse_rotation.rotate(transformation2.getPosition()),
           inverse_rotation);

  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, expected_transformation);

  intersected_polygon_owners.pop();

  EXPECT_EQ(intersected_polygon_owners.size(), 1)
      << "Wrong number of intersecting polygons.";

  result_id = 0;

  expected_transformation = transformation1 * transformation2;
  inverse_rotation = expected_transformation.getRotation().inverted();
  expected_transformation =
      Pose(-inverse_rotation.rotate(expected_transformation.getPosition()),
           inverse_rotation);

  test_id = std::get<0>(intersected_polygon_owners.front());
  test_transformation = std::get<1>(intersected_polygon_owners.front());

  EXPECT_EQ(test_id, result_id);
  EXPECT_EQ(test_transformation, expected_transformation);
}

TEST_F(PolygonConsolidationTest, GetPolygonUnion1) {
  PoseGraph pose_graph;

  std::vector<PolygonPoint> points1;
  points1.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  points1.emplace_back(0.0, 10.0, PointType::OBSTACLE);
  points1.emplace_back(10.0, 10.0, PointType::OBSTACLE);
  points1.emplace_back(10.0, -10.0, PointType::OBSTACLE);
  points1.emplace_back(0.0, -10.0, PointType::OBSTACLE);
  points1.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon polygon1(points1);

  pose_graph.addPose(Pose(), polygon1);

  kindr::AngleAxisD angleAxis2(1.5708 /*90deg*/, Eigen::Vector3d::UnitZ());
  Pose transformation(Position(5.0, 5.0, 0), Rotation(angleAxis2));

  std::vector<PolygonPoint> points2;
  points2.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  points2.emplace_back(0.0, 2.5, PointType::OBSTACLE);
  points2.emplace_back(10.0, 2.5, PointType::OBSTACLE);
  points2.emplace_back(10.0, -2.5, PointType::OBSTACLE);
  points2.emplace_back(0.0, -2.5, PointType::OBSTACLE);
  points2.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon polygon2(points2);

  pose_graph.addPose(transformation, polygon2);

  unsigned int reference_id = 0;
  auto intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(
          reference_id, pose_graph.getPoseGraphPoses());
  auto[polygon_union, intersected_polygon_owners_vector] =
      PolygonConsolidation::getPolygonUnion(reference_id,
                                            pose_graph.getPoseGraphPoses(),
                                            intersected_polygon_owners);

  auto expected_transformation = transformation;
  EXPECT_EQ(std::get<0>(intersected_polygon_owners_vector[0]), 1);
  EXPECT_EQ(std::get<1>(intersected_polygon_owners_vector[0]),
            expected_transformation);

  auto polygon_points = polygon_union.getPoints();

  EXPECT_EQ(polygon_points[0], PolygonPoint(2.49998, 10.0, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[1], PolygonPoint(2.49998, 15.0, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[2], PolygonPoint(7.49998, 15.0, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[3], PolygonPoint(7.49998, 10.0, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[4], PolygonPoint(10.0, 10.0, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[5], PolygonPoint(10.0, -10.0, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[6], PolygonPoint(0.0, -10.0, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[7], PolygonPoint(0.0, 0.0, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[8], PolygonPoint(0.0, 10.0, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[9], PolygonPoint(2.49998, 10.0, PointType::UNKNOWN));

  reference_id = 1;
  intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(
          reference_id, pose_graph.getPoseGraphPoses());
  std::tie(polygon_union, intersected_polygon_owners_vector) =
      PolygonConsolidation::getPolygonUnion(reference_id,
                                            pose_graph.getPoseGraphPoses(),
                                            intersected_polygon_owners);

  auto inverse_rotation = transformation.getRotation().inverted();
  expected_transformation = Pose(
      -inverse_rotation.rotate(transformation.getPosition()), inverse_rotation);
  EXPECT_EQ(std::get<0>(intersected_polygon_owners_vector[0]), 0);
  EXPECT_EQ(std::get<1>(intersected_polygon_owners_vector[0]),
            expected_transformation);

  polygon_points = polygon_union.getPoints();

  EXPECT_EQ(polygon_points[0], PolygonPoint(5.00001, 2.5, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[1], PolygonPoint(10, 2.5, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[2], PolygonPoint(10, -2.5, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[3], PolygonPoint(4.99999, -2.5, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[4],
            PolygonPoint(4.99998, -5.00002, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[5],
            PolygonPoint(-15.0, -4.99994, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[6],
            PolygonPoint(-15.0, 5.00006, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[7],
            PolygonPoint(-4.99998, 5.00002, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[8],
            PolygonPoint(5.00002, 4.99998, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[9], PolygonPoint(5.00001, 2.5, PointType::UNKNOWN));

  polygon_points = pose_graph.getPoseGraphPose(0).getPolygon().getPoints();
  auto polygon_edge_types =
      pose_graph.getPoseGraphPose(0).getPolygon().getEdgeTypes();

  EXPECT_EQ((polygon_points.size() - 1), polygon_edge_types.size())
      << "Number of edge should be one less than number of points!";

  for (const auto& polygon_edge_type : polygon_edge_types) {
    EXPECT_EQ(polygon_edge_type, EdgeType::OBSTACLE)
        << "Wrong polygon edge type";
  }

  auto first_pose_points =
      pose_graph.getPoseGraphPose(0).getPolygon().getPoints();
  for (const auto& polygon_point : first_pose_points) {
    EXPECT_EQ(polygon_point.getPointType(), PointType::OBSTACLE)
        << "Wrong point type";
  }

  auto second_pose_points =
      pose_graph.getPoseGraphPose(1).getPolygon().getPoints();
  EXPECT_EQ(second_pose_points[0].getPointType(), PointType::FREE_SPACE)
      << "Wrong point type";
  EXPECT_EQ(second_pose_points[1].getPointType(), PointType::FREE_SPACE)
      << "Wrong point type";
  EXPECT_EQ(second_pose_points[2].getPointType(), PointType::OBSTACLE)
      << "Wrong point type";
  EXPECT_EQ(second_pose_points[3].getPointType(), PointType::OBSTACLE)
      << "Wrong point type";
  EXPECT_EQ(second_pose_points[4].getPointType(), PointType::FREE_SPACE)
      << "Wrong point type";

  polygon_points = pose_graph.getPoseGraphPose(0).getPolygon().getPoints();
  polygon_edge_types =
      pose_graph.getPoseGraphPose(1).getPolygon().getEdgeTypes();

  EXPECT_EQ((polygon_points.size() - 1), polygon_edge_types.size())
      << "Number of edge should be one less than number of points!";

  EXPECT_EQ(polygon_edge_types.at(0), EdgeType::FREE_SPACE)
      << "Wrong polygon edge type";
  EXPECT_EQ(polygon_edge_types.at(1), EdgeType::FREE_SPACE)
      << "Wrong polygon edge type";
  EXPECT_EQ(polygon_edge_types.at(2), EdgeType::OBSTACLE)
      << "Wrong polygon edge type";
  EXPECT_EQ(polygon_edge_types.at(3), EdgeType::FREE_SPACE)
      << "Wrong polygon edge type";
  EXPECT_EQ(polygon_edge_types.at(4), EdgeType::FREE_SPACE)
      << "Wrong polygon edge type";
}

TEST_F(PolygonConsolidationTest, GetPolygonUnion2) {
  PoseGraph pose_graph;

  std::vector<PolygonPoint> points1;
  points1.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  points1.emplace_back(0.0, 10.0, PointType::OBSTACLE);
  points1.emplace_back(10.0, 10.0, PointType::OBSTACLE);
  points1.emplace_back(10.0, -10.0, PointType::OBSTACLE);
  points1.emplace_back(0.0, -10.0, PointType::OBSTACLE);
  points1.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon polygon1(points1);

  pose_graph.addPose(Pose(), polygon1);
  pose_graph.addPose(Pose(), polygon1);

  unsigned int reference_id = 0;
  auto intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(
          reference_id, pose_graph.getPoseGraphPoses());
  auto[polygon_union, intersected_polygon_owners_vector] =
      PolygonConsolidation::getPolygonUnion(reference_id,
                                            pose_graph.getPoseGraphPoses(),
                                            intersected_polygon_owners);

  auto expected_transformation = Pose();
  EXPECT_EQ(std::get<0>(intersected_polygon_owners_vector[0]), 1);
  EXPECT_EQ(std::get<1>(intersected_polygon_owners_vector[0]),
            expected_transformation);

  auto polygon_points = polygon_union.getPoints();

  EXPECT_EQ(polygon_points[0], PolygonPoint(0, 10, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[1], PolygonPoint(10, 10, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[2], PolygonPoint(10, -10, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[3], PolygonPoint(0, -10, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[4], PolygonPoint(0, 0, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[5], PolygonPoint(0, 10, PointType::UNKNOWN));

  reference_id = 1;
  intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(
          reference_id, pose_graph.getPoseGraphPoses());
  std::tie(polygon_union, intersected_polygon_owners_vector) =
      PolygonConsolidation::getPolygonUnion(reference_id,
                                            pose_graph.getPoseGraphPoses(),
                                            intersected_polygon_owners);

  EXPECT_EQ(std::get<0>(intersected_polygon_owners_vector[0]), 0);
  EXPECT_EQ(std::get<1>(intersected_polygon_owners_vector[0]),
            expected_transformation);

  polygon_points = polygon_union.getPoints();

  EXPECT_EQ(polygon_points[0], PolygonPoint(0, 10, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[1], PolygonPoint(10, 10, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[2], PolygonPoint(10, -10, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[3], PolygonPoint(0, -10, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[4], PolygonPoint(0, 0, PointType::UNKNOWN));
  EXPECT_EQ(polygon_points[5], PolygonPoint(0, 10, PointType::UNKNOWN));

  auto first_pose_points =
      pose_graph.getPoseGraphPose(0).getPolygon().getPoints();
  for (const auto& polygon_point : first_pose_points) {
    EXPECT_EQ(polygon_point.getPointType(), PointType::OBSTACLE)
        << "Wrong point type";
  }

  auto second_pose_points =
      pose_graph.getPoseGraphPose(1).getPolygon().getPoints();
  for (const auto& polygon_point : second_pose_points) {
    EXPECT_EQ(polygon_point.getPointType(), PointType::OBSTACLE)
        << "Wrong point type";
  }
}
