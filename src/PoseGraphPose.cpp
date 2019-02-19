/* Pose graph pose representation
 */

#include "PoseGraphPose.h"
#include <memory>
#include <utility>

PoseGraphPose::PoseGraphPose(Polygon& polygon, int pose_graph_pose_id)
    : id_(pose_graph_pose_id),
      polygon_(polygon) {
} /* -----  end of method PoseGraphPose::PoseGraphPose  (constructor)  ----- */

void PoseGraphPose::addAdjacentPose(
    int pose_id, std::shared_ptr<PoseGraphPose> pose_graph_pose,
    Pose& transformation) {
  if (0 == adjacentPoses_.count(pose_id)) {
    adjacentPoses_[pose_id] =
        std::make_tuple<std::shared_ptr<PoseGraphPose>, Pose>(
            std::move(pose_graph_pose), std::move(transformation));
  }
}
