/* Helper class for the polygon consolidation
 */

#include "PolygonConsolidation.h"
#include <glog/logging.h>
#include <tuple>
#include <queue>
#include <utility>
#include <vector>
#include <set>
#include <stack>

std::queue<std::tuple<unsigned int, Pose>>
PolygonConsolidation::getIntersectedPolygonOwners(
    unsigned int pose_graph_pose_id,
    std::vector<PoseGraphPose>& pose_graph_poses) {
  CHECK(0 <= pose_graph_pose_id) << "Id has to be non negativ!";
  CHECK(pose_graph_poses.size() > pose_graph_pose_id)
      << "Id has to be within range!";

  std::stack<std::tuple<unsigned int, unsigned int, Pose>> candidates;
  std::set<unsigned int> checked_candidates_id;
  std::queue<std::tuple<unsigned int, Pose>> intersected_polygon_owners;

  checked_candidates_id.insert(pose_graph_pose_id);

  // Get current pose and its polygon
  PoseGraphPose& current_pose_graph_pose = pose_graph_poses[pose_graph_pose_id];
  Polygon current_polygon = current_pose_graph_pose.getPolygon();

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
    auto current_previous_candidate_id = std::get<1>(current_candidate);
    auto current_candidate_to_referenct_transformation =
        std::get<2>(current_candidate);
    candidates.pop();

    /*
    LOG(INFO) << "Transformation from candidate (" << current_candidate_id
              << ") to previous candidate (" << current_previous_candidate_id
              << "): " << current_transformation;
    */

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
        LOG(INFO) << "transformation from parent ("
                  << std::get<1>(additional_candidate) << ") to reference ("
                  << pose_graph_pose_id << "): " << std::endl
                  << current_candidate_to_referenct_transformation;
        LOG(INFO) << "transformation from candidate ("
                  << std::get<0>(additional_candidate) << ") to parent ("
                  << std::get<1>(additional_candidate) << "): " << std::endl
                  << std::get<3>(additional_candidate);
        LOG(INFO) << "transformation from candidate ("
                  << std::get<0>(additional_candidate) << ")  to reference ("
                  << pose_graph_pose_id << ") : " << std::endl
                  << std::get<2>(additional_candidate);

        LOG(INFO) << "Added candidate (" << std::get<0>(additional_candidate)
                  << ") with parent (" << std::get<1>(additional_candidate)
                  << ") with transformation to reference ("
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

  return std::move(intersected_polygon_owners);
}

std::vector<std::tuple<unsigned int, unsigned int, Pose, Pose>>
PolygonConsolidation::addAdjacentCandidates(
    unsigned int current_candidate_id, Pose& current_transformation,
    std::set<unsigned int>& checked_candidates_id,
    std::vector<PoseGraphPose>& pose_graph_poses) {
  CHECK(0 <= current_candidate_id) << "Id has to be non negativ!";
  CHECK(pose_graph_poses.size() > current_candidate_id)
      << "Id has to be within range!";

  std::vector<std::tuple<unsigned int, unsigned int, Pose, Pose>> candidates;
  // Add adjacent poses and the corresponding transformation of
  // the current pose to the stack of candidates
  for (const auto& id :
       pose_graph_poses[current_candidate_id].getAdjacentPosesId()) {
    // Only add candidate if it is/was not yet a candidate
    if (0 == checked_candidates_id.count(id)) {
      auto candidate_to_parent_transformation =
          pose_graph_poses[id].getAdjacentPoses()[current_candidate_id];
      auto candidate_to_reference_transformation =
          candidate_to_parent_transformation * current_transformation;

      candidates.emplace_back(std::make_tuple(
          id, current_candidate_id, candidate_to_reference_transformation,
          candidate_to_parent_transformation));
      checked_candidates_id.insert(id);
    }
  }

  return std::move(candidates);
}
