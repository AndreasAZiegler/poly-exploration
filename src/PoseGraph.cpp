/* Pose graph representation
 */

#include "PoseGraph.h"
#include "PolygonConsolidation.h"
#include <glog/logging.h>
#include <memory>
#include <utility>
#include <vector>

PoseGraph::PoseGraph() : currentPoseGraphPoseId_(0) {}

void PoseGraph::addPose(const Polygon& polygon, const Pose transformation) {
  PoseGraphPose pose_graph_pose(polygon, currentPoseGraphPoseId_);

  poseGraphPoses_.emplace_back(pose_graph_pose);

  /*
  std::cout << "currentPoseGraphPoseId_: " << currentPoseGraphPoseId_
            << std::endl;
  */

  if (currentPoseGraphPoseId_ > 0) {
    auto inverse_rotation = transformation.getRotation().inverted();
    Pose inverted_transformation(
        -inverse_rotation.rotate(transformation.getPosition()),
        inverse_rotation);

    poseGraphPoses_[currentPoseGraphPoseId_ - 1].addAdjacentPose(
        currentPoseGraphPoseId_, transformation);

    LOG(INFO) << "Transformation between pose graph pose ("
              << currentPoseGraphPoseId_ - 1
              << ") and adjacent pose graph pose (" << currentPoseGraphPoseId_
              << "):" << std::endl
              << transformation;

    poseGraphPoses_.back().addAdjacentPose(currentPoseGraphPoseId_ - 1,
                                           inverted_transformation);

    LOG(INFO) << "Transformation between pose graph pose ("
              << currentPoseGraphPoseId_ << ") and adjacent pose graph pose ("
              << currentPoseGraphPoseId_ - 1 << "):" << std::endl
              << inverted_transformation;
  }

  // consolidatePolygon(currentPoseGraphPoseId_);

  currentPoseGraphPoseId_++;
}

void PoseGraph::consolidatePolygon(const unsigned int pose_graph_pose_id) {
  auto intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(pose_graph_pose_id,
                                                        poseGraphPoses_);
}

void PoseGraph::connectTwoPoses(unsigned int pose_id_1, unsigned int pose_id_2,
                                const Pose transformation) {
  auto inverse_rotation = transformation.getRotation().inverted();
  Pose inverted_transformation(
      -inverse_rotation.rotate(transformation.getPosition()), inverse_rotation);

  poseGraphPoses_[pose_id_1].addAdjacentPose(pose_id_2, transformation);

  poseGraphPoses_[pose_id_2].addAdjacentPose(pose_id_1,
                                             inverted_transformation);
}

std::vector<PoseGraphPose> PoseGraph::getPoseGraphPoses() const {
  return std::move(poseGraphPoses_);
}

PoseGraphPose PoseGraph::getPoseGraphPose() const {
  CHECK(!poseGraphPoses_.empty())
      << "There should be at least one pose in the pose graph!";
  /*
  auto pose_graph = poseGraphPoses_.back();

  if (!pose_graph.getAdjacentPoses().empty()) {
    std::cout << "PoseGraph::getPoseGraphPose(void): Transformation: "
              << std::get<1>(pose_graph.getAdjacentPoses()[0]) << std::endl;
  }

  return pose_graph;
  */
  return poseGraphPoses_.back();
}

PoseGraphPose PoseGraph::getPoseGraphPose(const unsigned int id) const {
  CHECK(poseGraphPoses_.size() > id) << "Id has to be within range!";
  /*
  auto& pose_graph = poseGraphPoses_.at(id);

  std::cout << "PoseGraph::getPoseGraphPose(void): Transformation: "
            << std::get<1>(pose_graph.getAdjacentPoses()[0]) << std::endl;

  return pose_graph;
  */
  return poseGraphPoses_.at(id);
}
