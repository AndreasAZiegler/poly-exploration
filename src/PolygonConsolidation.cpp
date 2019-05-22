/* Helper class for the polygon consolidation
 */

#include "poly_exploration/PolygonConsolidation.h"
#include <glog/logging.h>
#include <queue>
#include <set>
#include <stack>
#include <tuple>
#include <utility>
#include <vector>

std::tuple<Polygon, std::vector<std::tuple<unsigned int, Pose>>>
PolygonConsolidation::getPolygonUnion(
    const unsigned int pose_graph_pose_id,
    const std::vector<PoseGraphPose>& pose_graph_poses,
    std::queue<std::tuple<unsigned int, Pose>>* intersected_polygon_owners) {
  CHECK(pose_graph_poses.size() > pose_graph_pose_id)
      << "Id has to be within range!";

  std::vector<std::tuple<unsigned int, Pose>> intersected_polygon_owners_vector;

  LOG(INFO) << "Reference id (" << pose_graph_pose_id << ") has "
            << intersected_polygon_owners->size() << " intersecting polygons.";

  auto polygon_union = pose_graph_poses[pose_graph_pose_id].getPolygon();
  LOG(INFO) << "Reference polygon:" << std::endl << polygon_union;
  polygon_union.plot("plot-start.svg");

  while (!intersected_polygon_owners->empty()) {
    auto intersected_polygon_owner = intersected_polygon_owners->front();
    intersected_polygon_owners_vector.emplace_back(intersected_polygon_owner);

    auto candidate_id = std::get<0>(intersected_polygon_owner);
    auto candidate_to_reference_transformation =
        std::get<1>(intersected_polygon_owner);

    LOG(INFO) << "candidate id: " << candidate_id;

    auto candidate_polygon_origin_frame =
        pose_graph_poses[candidate_id].getPolygon();
    auto candidate_polygon_reference_frame =
        candidate_polygon_origin_frame.transformPolygon(
            candidate_to_reference_transformation);

    polygon_union.plot("plot-" + std::to_string(candidate_id) + "-before.svg");
    LOG(INFO) << "Number of points BEFORE union: "
              << polygon_union.getPoints().size();
    LOG(INFO) << "polygon_union BEFORE:" << std::endl << polygon_union;
    LOG(INFO) << "candidate polygon in the candidate frame:" << std::endl
              << candidate_polygon_origin_frame;
    LOG(INFO) << "candidate polygon in the reference frame:" << std::endl
              << candidate_polygon_reference_frame;

    polygon_union = polygon_union.buildUnion(candidate_polygon_reference_frame);
    LOG(INFO) << "polygon_union AFTER:" << std::endl << polygon_union;

    polygon_union.plot("plot-" + std::to_string(candidate_id) + "-after.svg");
    LOG(INFO) << "Number of points AFTER union: "
              << polygon_union.getPoints().size();

    intersected_polygon_owners->pop();
  }
  polygon_union.plot("plot-end.svg");

  return std::make_tuple(polygon_union, intersected_polygon_owners_vector);
}

std::queue<std::tuple<unsigned int, Pose>>
PolygonConsolidation::getIntersectedPolygonOwners(
    const unsigned int pose_graph_pose_id,
    const std::vector<PoseGraphPose>& pose_graph_poses) {
  CHECK(pose_graph_poses.size() > pose_graph_pose_id)
      << "Id has to be within range!";

  std::stack<std::tuple<unsigned int, unsigned int, Pose>> candidates;
  std::set<unsigned int> checked_candidates_id;
  std::queue<std::tuple<unsigned int, Pose>> intersected_polygon_owners;

  checked_candidates_id.insert(pose_graph_pose_id);

  // Get current pose and its polygon
  auto current_pose_graph_pose = pose_graph_poses[pose_graph_pose_id];
  auto current_polygon = current_pose_graph_pose.getPolygon();

  // Add adjacent poses and the corresponding transformation of the current pose
  // This initializes the candidates stack
  Pose identity_transformation;
  auto initial_candidates = PolygonConsolidation::addAdjacentCandidates(
      pose_graph_pose_id, identity_transformation, checked_candidates_id,
      pose_graph_poses);

  for (const auto& initial_candidate : initial_candidates) {
    candidates.emplace(std::make_tuple(
        std::get<0>(initial_candidate),  // candidate id
        std::get<1>(initial_candidate),  // parent id
        std::get<2>(
            initial_candidate)));  // candidate to reference transformation
  }

  // Continue until there are no candidates left
  while (!candidates.empty()) {
    // Get current candidate
    auto current_candidate = candidates.top();
    auto current_candidate_id = std::get<0>(current_candidate);
    auto current_candidate_to_referenct_transformation =
        std::get<2>(current_candidate);
    candidates.pop();

    // Transform candidate polygon into current pose's frame
    auto other_polygon = pose_graph_poses[current_candidate_id].getPolygon();
    auto other_polygon_transformed = other_polygon.transformPolygon(
        current_candidate_to_referenct_transformation);

    // Check if candidate intersect with the current polygon
    if (current_polygon.checkForIntersections(other_polygon_transformed)) {
      // Add current polygon and the corresponding transformation into
      // the queue of intersection polygons
      intersected_polygon_owners.emplace(
          current_candidate_id, current_candidate_to_referenct_transformation);

      // Add adjacent poses and the corresponding transformation of
      // the current pose to the stack of candidates
      auto additional_candidates = PolygonConsolidation::addAdjacentCandidates(
          current_candidate_id, current_candidate_to_referenct_transformation,
          checked_candidates_id, pose_graph_poses);

      for (const auto& additional_candidate : additional_candidates) {
        LOG(INFO) << "Transformation calculations: candidate ("
                  << std::get<0>(additional_candidate) << ") with parent ("
                  << std::get<1>(additional_candidate) << ")";
        LOG(INFO) << "transformation from reference (" << pose_graph_pose_id
                  << ") to parent (" << std::get<1>(additional_candidate)
                  << "):" << std::endl
                  << current_candidate_to_referenct_transformation;
        LOG(INFO) << "transformation from parent ("
                  << std::get<1>(additional_candidate) << ") to candidate ("
                  << std::get<0>(additional_candidate) << "):" << std::endl
                  << std::get<3>(additional_candidate);
        LOG(INFO) << "transformation from reference (" << pose_graph_pose_id
                  << ") to candidate (" << std::get<0>(additional_candidate)
                  << "):" << std::endl
                  << std::get<2>(additional_candidate);

        LOG(INFO) << "Added candidate (" << std::get<0>(additional_candidate)
                  << ") with parent (" << std::get<1>(additional_candidate)
                  << ") with transformation from reference ("
                  << pose_graph_pose_id << "): " << std::endl
                  << std::get<2>(additional_candidate);
        candidates.emplace(std::make_tuple(
            std::get<0>(additional_candidate),    // candidate id
            std::get<1>(additional_candidate),    // parent id
            std::get<2>(additional_candidate)));  // candidate to reference
                                                  // transformation
      }
    }
  }

  return intersected_polygon_owners;
}

std::vector<std::tuple<unsigned int, unsigned int, Pose, Pose>>
PolygonConsolidation::addAdjacentCandidates(
    const unsigned int current_candidate_id, const Pose& current_transformation,
    std::set<unsigned int>& checked_candidates_id,
    const std::vector<PoseGraphPose>& pose_graph_poses) {
  CHECK(pose_graph_poses.size() > current_candidate_id)
      << "Id has to be within range!";

  std::vector<std::tuple<unsigned int, unsigned int, Pose, Pose>> candidates;
  // Add adjacent poses and the corresponding transformation of
  // the current pose to the stack of candidates
  for (const auto& id :
       pose_graph_poses[current_candidate_id].getAdjacentPosesId()) {
    // Only add candidate if it is/was not yet a candidate
    if (0 == checked_candidates_id.count(id)) {
      auto adjacent_transformations =
          pose_graph_poses[current_candidate_id].getAdjacentPoses();
      auto parent_to_candidate_transformation = adjacent_transformations[id];
      auto reference_to_candidate_transformation =
          current_transformation * parent_to_candidate_transformation;

      candidates.emplace_back(std::make_tuple(
          id, current_candidate_id, reference_to_candidate_transformation,
          parent_to_candidate_transformation));
      checked_candidates_id.insert(id);
    }
  }

  return candidates;
}

void PolygonConsolidation::setAllIntersectedPolygonsToPerformUnion(
    std::vector<std::tuple<unsigned int, Pose>>& intersected_polygon_owners,
    std::vector<PoseGraphPose>* pose_graph_poses) {
  // Set all points of intersected polygons to UNKNOWN
  for (const auto& intersected_polygon_owner : intersected_polygon_owners) {
    auto intersected_polygon_pose_id = std::get<0>(intersected_polygon_owner);
    pose_graph_poses->at(intersected_polygon_pose_id)
        .setPolygonPointsToPerformUnion();
  }
}

void PolygonConsolidation::setMaxRangeAndObstaclePoints(
    std::vector<PoseGraphPose>* pose_graph_poses, const Polygon& polygon_union,
    std::vector<std::tuple<unsigned int, Pose>>& intersected_polygon_owners) {
  auto polygon_union_points = polygon_union.getPoints();

  for (const auto& polygon_union_point : polygon_union_points) {
    for (const auto& intersected_polygon_owner : intersected_polygon_owners) {
      auto intersected_polygon_pose_id = std::get<0>(intersected_polygon_owner);
      auto reference_to_candidate_transformation =
          std::get<1>(intersected_polygon_owner);
      auto intersected_polygon =
          pose_graph_poses->at(intersected_polygon_pose_id).getPolygon();

      auto intersected_polygon_reference_frame =
          intersected_polygon.transformPolygon(
              reference_to_candidate_transformation);

      auto intersected_polygon_points_reference_frame =
          intersected_polygon_reference_frame.getPoints();

      for (unsigned int intersected_polygon_point_reference_frame_id = 0;
           intersected_polygon_point_reference_frame_id <
           intersected_polygon_points_reference_frame.size();
           ++intersected_polygon_point_reference_frame_id) {
        auto intersected_polygon_point_reference_frame =
            intersected_polygon_points_reference_frame
                [intersected_polygon_point_reference_frame_id];
        if (static_cast<Point>(intersected_polygon_point_reference_frame) ==
            static_cast<Point>(polygon_union_point)) {
          LOG(INFO) << "Pose id: " << intersected_polygon_pose_id << std::endl
                    << "Point id: "
                    << intersected_polygon_point_reference_frame_id;
          if (intersected_polygon_point_reference_frame.getPointType() ==
              PointType::WAS_MAX_RANGE) {
            pose_graph_poses->at(intersected_polygon_pose_id)
                .setPolygonPointType(
                    intersected_polygon_point_reference_frame_id,
                    PointType::MAX_RANGE);
            LOG(INFO) << "Set MAX_RANGE";
          } else if (intersected_polygon_point_reference_frame.getPointType() ==
                     PointType::WAS_OBSTACLE) {
            pose_graph_poses->at(intersected_polygon_pose_id)
                .setPolygonPointType(
                    intersected_polygon_point_reference_frame_id,
                    PointType::OBSTACLE);
            LOG(INFO) << "Set OBSTACLE";
          }
        }
      }
    }
  }
}

void PolygonConsolidation::setFreeSpacePoints(
    std::vector<PoseGraphPose>* pose_graph_poses,
    std::vector<std::tuple<unsigned int, Pose>>& intersected_polygon_owners) {
  for (const auto& intersected_polygon_owner : intersected_polygon_owners) {
    auto intersected_polygon_pose_id = std::get<0>(intersected_polygon_owner);
    auto intersected_polygon =
        pose_graph_poses->at(intersected_polygon_pose_id).getPolygon();

    auto intersected_polygon_points_reference_frame =
        intersected_polygon.getPoints();

    for (unsigned int intersected_polygon_point_reference_frame_id = 0;
         intersected_polygon_point_reference_frame_id <
         intersected_polygon_points_reference_frame.size();
         ++intersected_polygon_point_reference_frame_id) {
      auto intersected_polygon_point_reference_frame =
          intersected_polygon_points_reference_frame
              [intersected_polygon_point_reference_frame_id];
      if ((intersected_polygon_point_reference_frame.getPointType() ==
           PointType::WAS_MAX_RANGE) ||
          (intersected_polygon_point_reference_frame.getPointType() ==
           PointType::WAS_OBSTACLE)) {
        LOG(INFO)
            << "Pose id: " << intersected_polygon_pose_id << std::endl
            << "Point id: " << intersected_polygon_point_reference_frame_id
            << std::endl
            << "Point type: "
            << static_cast<int>(
                   intersected_polygon_point_reference_frame.getPointType());
        pose_graph_poses->at(intersected_polygon_pose_id)
            .setPolygonPointType(intersected_polygon_point_reference_frame_id,
                                 PointType::FREE_SPACE);
        LOG(INFO) << "Set FREE_SPACE";
      }
    }
  }
}

