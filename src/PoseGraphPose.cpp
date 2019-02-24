/* Pose graph pose representation
 */

#include "PoseGraphPose.h"
#include <memory>
#include <utility>

PoseGraphPose::PoseGraphPose(Polygon& polygon, unsigned int pose_graph_pose_id)
    : id_(pose_graph_pose_id), polygon_(polygon) {
} /* -----  end of method PoseGraphPose::PoseGraphPose  (constructor)  ----- */

void PoseGraphPose::addAdjacentPose(unsigned int pose_graph_pose_id,
                                    Pose& transformation) {
  adjacentPoses_[pose_graph_pose_id] = std::move(transformation);
}

std::vector<unsigned int> PoseGraphPose::getAdjacentPosesId() {
  std::vector<unsigned int> adjacent_pose_graph_pose_ids;
  adjacent_pose_graph_pose_ids.reserve(adjacentPoses_.size());

  for (const auto& adjacent_pose : adjacentPoses_) {
    adjacent_pose_graph_pose_ids.push_back(adjacent_pose.first);
  }

  return adjacent_pose_graph_pose_ids;
}
