/* Pose graph representation
 */

#include "PoseGraph.h"
#include <memory>

PoseGraph::PoseGraph() : currentPoseGraphPoseId_(0) {}

void PoseGraph::addPose(Polygon& polygon, Pose transformation) {
  PoseGraphPose pose_graph_pose(polygon, currentPoseGraphPoseId_);

  poseGraphPoses_.emplace_back(pose_graph_pose);

  if (currentPoseGraphPoseId_ > 0) {
    auto inverse_rotation = transformation.getRotation().inverted();
    Pose inverted_transformation(
        -inverse_rotation.rotate(transformation.getPosition()),
        inverse_rotation);
    poseGraphPoses_[currentPoseGraphPoseId_].addAdjacentPose(
        currentPoseGraphPoseId_ + 1,
        std::make_shared<PoseGraphPose>(pose_graph_pose),
        inverted_transformation);

    poseGraphPoses_.back().addAdjacentPose(
        currentPoseGraphPoseId_,
        std::make_shared<PoseGraphPose>(
            poseGraphPoses_[currentPoseGraphPoseId_]),
        transformation);
  }

  currentPoseGraphPoseId_++;
}

PoseGraphPose& PoseGraph::getPoseGraphPose(void) {
  return poseGraphPoses_.back();
}

PoseGraphPose& PoseGraph::getPoseGraphPose(const unsigned int id) {
  return poseGraphPoses_.at(id);
}
