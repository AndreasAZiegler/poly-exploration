/* Pose graph pose representation
 */

#include "PoseGraphPose.h"
#include <memory>
#include <utility>

PoseGraphPose::PoseGraphPose(Polygon& polygon, unsigned int pose_graph_pose_id)
    : id_(pose_graph_pose_id),
      polygon_(polygon),
      polygonEdgeTypes_(polygon.getPoints().size()) {
  for (auto& edge_type : polygonEdgeTypes_) {

  }
} /* -----  end of method PoseGraphPose::PoseGraphPose  (constructor)  ----- */

void PoseGraphPose::addAdjacentPose(unsigned int pose_graph_pose_id,
                                    Pose& transformation) {
  adjacentPoses_[pose_graph_pose_id] = std::move(transformation);
}
