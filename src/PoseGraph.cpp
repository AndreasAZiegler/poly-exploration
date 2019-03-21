/* Pose graph representation
 */

#include "PoseGraph.h"
#include <glog/logging.h>
#include <memory>
#include <utility>
#include <vector>
#include "PolygonConsolidation.h"

PoseGraph::PoseGraph() : currentPoseGraphPoseId_{0} {}

void PoseGraph::addPose(
    const Pose& current_pose_to_previews_pose_transformation,
    const Polygon& polygon) {
  PoseGraphPose pose_graph_pose(currentPoseGraphPoseId_, polygon);

  poseGraphPoses_.emplace_back(pose_graph_pose);

  if (currentPoseGraphPoseId_ > 0) {
    auto inverse_rotation =
        current_pose_to_previews_pose_transformation.getRotation().inverted();
    Pose inverted_transformation(
        -inverse_rotation.rotate(
            current_pose_to_previews_pose_transformation.getPosition()),
        inverse_rotation);

    poseGraphPoses_[currentPoseGraphPoseId_ - 1].addAdjacentPose(
        currentPoseGraphPoseId_, current_pose_to_previews_pose_transformation);

    LOG(INFO) << "Transformation between pose graph pose ("
              << currentPoseGraphPoseId_ - 1
              << ") and adjacent pose graph pose (" << currentPoseGraphPoseId_
              << "):" << std::endl
              << current_pose_to_previews_pose_transformation;

    poseGraphPoses_.back().addAdjacentPose(currentPoseGraphPoseId_ - 1,
                                           inverted_transformation);

    LOG(INFO) << "Transformation between pose graph pose ("
              << currentPoseGraphPoseId_ << ") and adjacent pose graph pose ("
              << currentPoseGraphPoseId_ - 1 << "):" << std::endl
              << inverted_transformation;

    consolidatePolygon(currentPoseGraphPoseId_);
  }

  currentPoseGraphPoseId_++;
}

void PoseGraph::consolidatePolygon(const unsigned int pose_graph_pose_id) {
  // Get required data
  auto intersected_polygon_owners =
      PolygonConsolidation::getIntersectedPolygonOwners(pose_graph_pose_id,
                                                        poseGraphPoses_);
  auto[polygon_union, intersected_polygon_owners_vector] =
      PolygonConsolidation::getPolygonUnion(pose_graph_pose_id, poseGraphPoses_,
                                            intersected_polygon_owners);

  // Add current pose (pose_graph_pose_id) to the intersected polygon
  // owners as we also want to consolidate the polygon points of the
  // current pose
  intersected_polygon_owners_vector.emplace_back(
      std::make_tuple(pose_graph_pose_id, Pose()));

  // Set all points of intersected polygons to UNKNOWN
  PolygonConsolidation::setAllIntersectedPolygonsToPerformUnion(
      intersected_polygon_owners_vector, &poseGraphPoses_);

  // Set points MAX_RANGE or OBSTACLE which are contained in the
  // polygon union.
  PolygonConsolidation::setMaxRangeAndObstaclePoints(
      &poseGraphPoses_, polygon_union, intersected_polygon_owners_vector);

  // Set points which are not MAX_RANGE or OBSTACLE to FREE_SPACE. These
  // are points which were not in the unifyed polygon and therefore must
  // represent polygon point in free space
  PolygonConsolidation::setFreeSpacePoints(&poseGraphPoses_, polygon_union,
                                           intersected_polygon_owners_vector);

  // Determine and set edge types of the intersected polygons
  for (const auto& intersected_polygon_owner :
       intersected_polygon_owners_vector) {
    auto intersected_polygon_pose_id = std::get<0>(intersected_polygon_owner);
    poseGraphPoses_[intersected_polygon_pose_id].determinePolygonEdgeTypes();
  }
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
