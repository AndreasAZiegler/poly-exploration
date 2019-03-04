/* Helper class for the polygon consolidation
 */

#include "PolygonConsolidation.h"
#include <glog/logging.h>

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
  Polygon& current_polygon = current_pose_graph_pose.getPolygon();

  // Add adjacent poses and the corresponding transformation of the current pose
  // This initializes the candidates stack
  Pose identity_transformation;
  PolygonConsolidation::addAdjacentCandidates(
      pose_graph_pose_id, identity_transformation, candidates,
      checked_candidates_id, pose_graph_poses);

  // Continue until there are no candidates left
  while (!candidates.empty()) {
    // Get current candidate
    auto current_candidate = candidates.top();
    auto current_candidate_id = std::get<0>(current_candidate);
    auto current_previous_candidate_id = std::get<1>(current_candidate);
    // auto current_previous_transformation = std::get<2>(current_candidate);
    candidates.pop();

    // Get adjacent transformation
    auto adjacent_poses =
        pose_graph_poses[current_candidate_id].getAdjacentPoses();
    auto current_transformation = adjacent_poses[current_previous_candidate_id];

    /*
    Pose current_transformation =
        current_previous_transformation * adjacent_transformation;
    */

    std::cout << "current_transformation: " << current_transformation
              << std::endl;

    // Transform candidate polygon into current pose's frame
    auto other_polygon = pose_graph_poses[current_candidate_id].getPolygon();
    auto other_polygon_transformed =
        other_polygon.transformPolygon(current_transformation);

    // Check if candidate intersect with the current polygon
    if (current_polygon.checkForIntersections(other_polygon_transformed)) {
      // Add current polygon and the corresponding transformation into
      // the queue of intersection polygons
      intersected_polygon_owners.emplace(current_candidate_id,
                                         current_transformation);

      // Add adjacent poses and the corresponding transformation of
      // the current pose to the stack of candidates
      PolygonConsolidation::addAdjacentCandidates(
          current_candidate_id, current_transformation, candidates,
          checked_candidates_id, pose_graph_poses);
    }
  }

  return std::move(intersected_polygon_owners);
}

void PolygonConsolidation::addAdjacentCandidates(
    unsigned int current_candidate_id, Pose& current_transformation,
    std::stack<std::tuple<unsigned int, unsigned int, Pose>>& candidates,
    std::set<unsigned int>& checked_candidates_id,
    std::vector<PoseGraphPose>& pose_graph_poses) {
  CHECK(0 <= current_candidate_id) << "Id has to be non negativ!";
  CHECK(pose_graph_poses.size() > current_candidate_id)
      << "Id has to be within range!";
  // Add adjacent poses and the corresponding transformation of
  // the current pose to the stack of candidates
  for (const auto& id :
       pose_graph_poses[current_candidate_id].getAdjacentPosesId()) {
    // Only add candidate if it is/was not yet a candidate
    if (0 == checked_candidates_id.count(id)) {
      auto adjacent_poses = pose_graph_poses[id].getAdjacentPoses();

      std::cout << "Transformation calculations: current_candidate_id: "
                << current_candidate_id << std::endl;
      std::cout << "current_transformation: " << current_transformation
                << std::endl;
      std::cout << "adjacent_poses[current_candidate_id]: "
                << adjacent_poses[current_candidate_id] << std::endl;
      auto adjacent_transformation =
          current_transformation * adjacent_poses[current_candidate_id];
      std::cout << "adjacent_transformation: " << adjacent_transformation
                << std::endl;

      candidates.emplace(
          std::make_tuple(id, current_candidate_id, adjacent_transformation));
      checked_candidates_id.insert(id);
    }
  }
}
