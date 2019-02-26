/* Pose graph pose representation
 */

#include "PoseGraphPose.h"
#include <memory>
#include <utility>

PoseGraphPose::PoseGraphPose(const Polygon& polygon,
                             unsigned int pose_graph_pose_id)
    : id_(pose_graph_pose_id),
      polygon_(polygon) {
} /* -----  end of method PoseGraphPose::PoseGraphPose  (constructor)  ----- */

void PoseGraphPose::addAdjacentPose(unsigned int pose_graph_pose_id,
                                    const Pose& transformation) {
  adjacentPoses_[pose_graph_pose_id] = transformation;
}

Polygon& PoseGraphPose::getPolygon() { return polygon_; }

std::map<unsigned int, Pose> PoseGraphPose::getAdjacentPoses() {
  return adjacentPoses_;
}

std::vector<unsigned int> PoseGraphPose::getAdjacentPosesId() {
  std::vector<unsigned int> adjacent_pose_graph_pose_ids;
  adjacent_pose_graph_pose_ids.reserve(adjacentPoses_.size());

  for (const auto& adjacent_pose : adjacentPoses_) {
    adjacent_pose_graph_pose_ids.push_back(adjacent_pose.first);
  }

  return adjacent_pose_graph_pose_ids;
}

int PoseGraphPose::getId() { return id_; }
