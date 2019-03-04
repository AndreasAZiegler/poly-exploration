/* Helper class for the polygon consolidation
 */

#pragma once

#include <queue>
#include <tuple>
#include "Geometry.h"
#include "PoseGraph.h"

class PolygonConsolidation {
 public:
  PolygonConsolidation() = default;

  static std::queue<std::tuple<unsigned int, Pose>> getIntersectedPolygonOwners(
      unsigned int pose_graph_pose_id,
      std::vector<PoseGraphPose>& pose_graph_poses);

  static void addAdjacentCandidates(
      unsigned int current_candidate_id, Pose& current_transformation,
      std::stack<std::tuple<unsigned int, unsigned int, Pose>>& candidates,
      std::set<unsigned int>& checked_candidates_id,
      std::vector<PoseGraphPose>& pose_graph_poses);
}; /* -----  end of class PolygonConsolidation  ----- */

