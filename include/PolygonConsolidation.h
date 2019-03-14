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

  static std::tuple<Polygon, std::vector<std::tuple<unsigned int, Pose>>>
  getPolygonUnion(
      const unsigned int pose_graph_pose_id,
      const std::vector<PoseGraphPose>& pose_graph_poses,
      std::queue<std::tuple<unsigned int, Pose>> intersected_polygon_owners);

  static std::queue<std::tuple<unsigned int, Pose>> getIntersectedPolygonOwners(
      const unsigned int pose_graph_pose_id,
      const std::vector<PoseGraphPose>& pose_graph_poses);

  static std::vector<std::tuple<unsigned int, unsigned int, Pose, Pose>>
  addAdjacentCandidates(const unsigned int current_candidate_id,
                        const Pose& current_transformation,
                        std::set<unsigned int>& checked_candidates_id,
                        const std::vector<PoseGraphPose>& pose_graph_poses);

  static void setAllIntersectedPolygonsToPerformUnion(
      std::vector<std::tuple<unsigned int, Pose>>& intersected_polygon_owners,
      std::vector<PoseGraphPose>* pose_graph_poses);

  static void setMaxRangeAndObstaclePoints(
      std::vector<PoseGraphPose>* pose_graph_poses,
      const Polygon& polygon_union,
      std::vector<std::tuple<unsigned int, Pose>>& intersected_polygon_owners);

  static void setFreeSpacePoints(
      std::vector<PoseGraphPose>* pose_graph_poses,
      const Polygon& polygon_union,
      std::vector<std::tuple<unsigned int, Pose>>& intersected_polygon_owners);
}; /* -----  end of class PolygonConsolidation  ----- */

