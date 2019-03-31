/* Pose graph pose representation
 */

#include "poly_exploration/PoseGraphPose.h"
#include <memory>
#include <utility>

PoseGraphPose::PoseGraphPose(unsigned int pose_graph_pose_id,
                             const Polygon& polygon)
    : id_(pose_graph_pose_id),
      polygon_(polygon) {
} /* -----  end of method PoseGraphPose::PoseGraphPose  (constructor)  ----- */

void PoseGraphPose::addAdjacentPose(unsigned int pose_graph_pose_id,
                                    const Pose& transformation) {
  adjacentPoses_[pose_graph_pose_id] = transformation;
}

Polygon PoseGraphPose::getPolygon() const { return polygon_; }

void PoseGraphPose::setPolygonPointsToPerformUnion() {
  polygon_.setPointTypesToPerformUnion();
}

void PoseGraphPose::setPolygonPointType(unsigned int polygon_point_id,
                                        PointType point_type) {
  polygon_.setPointType(polygon_point_id, point_type);
}

void PoseGraphPose::determinePolygonEdgeTypes() {
  polygon_.determinePolygonEdgeTypes();
}

std::map<unsigned int, Pose> PoseGraphPose::getAdjacentPoses() const {
  return adjacentPoses_;
}

std::vector<unsigned int> PoseGraphPose::getAdjacentPosesId() const {
  std::vector<unsigned int> adjacent_pose_graph_pose_ids;
  adjacent_pose_graph_pose_ids.reserve(adjacentPoses_.size());

  for (const auto& adjacent_pose : adjacentPoses_) {
    adjacent_pose_graph_pose_ids.push_back(adjacent_pose.first);
  }

  return adjacent_pose_graph_pose_ids;
}

int PoseGraphPose::getId() const { return id_; }

std::ostream& operator<<(std::ostream& os,
                         const PoseGraphPose& pose_graph_pose) {
  os << "PoseGraphPose:" << std::endl
     << "Id: " << pose_graph_pose.getId() << std::endl;

  for (auto const & [ key, val ] : pose_graph_pose.getAdjacentPoses()) {
    os << "Adjacent pose " << key << " transformation: " << val << std::endl;
  }

  return os;
}
