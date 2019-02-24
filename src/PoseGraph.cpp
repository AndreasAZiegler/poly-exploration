/* Pose graph representation
 */

#include "PoseGraph.h"
#include <memory>

PoseGraph::PoseGraph() : currentPoseGraphPoseId_(0) {}

void PoseGraph::addPose(Polygon& polygon, Pose transformation) {
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

    /*
    std::cout << "Pose graph pose: " << currentPoseGraphPoseId_ - 1
              << " adjacent pose graph pose: " << currentPoseGraphPoseId_
              << std::endl;

    std::cout << "PoseGraph::addPose(): Transformation: " << transformation
              << std::endl;

    std::cout << "PoseGraph::addPose(): From Map Transformation: "
              << std::get<1>(poseGraphPoses_[currentPoseGraphPoseId_ - 1]
                                 .getAdjacentPoses()[currentPoseGraphPoseId_])
              << std::endl;
    */

    poseGraphPoses_.back().addAdjacentPose(currentPoseGraphPoseId_ - 1,
                                           inverted_transformation);

    /*
    std::cout << "Pose graph pose: " << currentPoseGraphPoseId_
              << " adjacent pose graph pose: " << currentPoseGraphPoseId_ - 1
              << std::endl;

    std::cout << "PoseGraph::addPose(): Inverted transformation: "
              << inverted_transformation << std::endl;

    std::cout << "PoseGraph::addPose(): From Map Inverted transformation: "
              << std::get<1>(poseGraphPoses_.back()
                                 .getAdjacentPoses()[currentPoseGraphPoseId_])
              << std::endl;
    */
  }

  currentPoseGraphPoseId_++;
}

void PoseGraph::consolidatePolygon(unsigned int pose_graph_pose_id) {
  std::queue<unsigned int> intersected_polygon_owners;
  std::stack<unsigned int> candidate_polygon_owners;

  PoseGraphPose& current_pose_graph_pose = poseGraphPoses_[pose_graph_pose_id];
  Polygon& current_polygon = current_pose_graph_pose.getPolygon();

  for (const auto& id :
       poseGraphPoses_[pose_graph_pose_id].getAdjacentPosesId()) {
    candidate_polygon_owners.emplace(id);
  }

  auto previous_pose_graph_pose_id = pose_graph_pose_id;
  Pose transformation;

  while (!intersected_polygon_owners.empty()) {
    auto current_pose_graph_pose_id = candidate_polygon_owners.top();
    auto adjacent_poses = poseGraphPoses_[current_pose_graph_pose_id].getAdjacentPoses();
    auto adjacent_transformation = adjacent_poses[previous_pose_graph_pose_id];
    transformation = transformation * adjacent_transformation;

    auto other_polygon =
        poseGraphPoses_[current_pose_graph_pose_id].getPolygon();
    auto other_polygon_transformed = other_polygon.transformPolygon(transformation);

    if (current_polygon.checkForIntersections(other_polygon)) {
    }

    previous_pose_graph_pose_id = current_pose_graph_pose_id;
  }
}

void PoseGraph::connectTwoPoses(unsigned int pose_id_1, unsigned int pose_id_2,
                                Pose transformation) {
  auto inverse_rotation = transformation.getRotation().inverted();
  Pose inverted_transformation(
      -inverse_rotation.rotate(transformation.getPosition()), inverse_rotation);

  poseGraphPoses_[pose_id_1].addAdjacentPose(pose_id_2, transformation);

  poseGraphPoses_[pose_id_2].addAdjacentPose(pose_id_1,
                                             inverted_transformation);
}

PoseGraphPose& PoseGraph::getPoseGraphPose(void) {
  auto& pose_graph = poseGraphPoses_.back();

  /*
  if (!pose_graph.getAdjacentPoses().empty()) {
    std::cout << "PoseGraph::getPoseGraphPose(void): Transformation: "
              << std::get<1>(pose_graph.getAdjacentPoses()[0]) << std::endl;
  }
  */

  return pose_graph;
}

PoseGraphPose& PoseGraph::getPoseGraphPose(const unsigned int id) {
  auto& pose_graph = poseGraphPoses_.at(id);

  /*
  std::cout << "PoseGraph::getPoseGraphPose(void): Transformation: "
            << std::get<1>(pose_graph.getAdjacentPoses()[0]) << std::endl;
  */

  return pose_graph;
}
