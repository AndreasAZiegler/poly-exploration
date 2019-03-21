/* Polygon explorer (main) class representation
 */

#include "PolygonExplorer.h"

PolygonExplorer::PolygonExplorer() {} /* -----  end of method
                                         PolygonExplorer::PolygonExplorer
                                         (constructor)  ----- */

void PolygonExplorer::setCallBack(std::function<void(const PoseGraph)> callback_function) {
  callBackList_.append(callback_function);
}

void PolygonExplorer::addPose(
    const Pose& current_pose_to_previews_pose_transformation,
    const Polygon& polygon) {
  poseGraph_.addPose(current_pose_to_previews_pose_transformation, polygon);

  callBackList_(poseGraph_);
}

std::vector<PoseGraphPose> PolygonExplorer::getPoseGraphPoses() const {
  return poseGraph_.getPoseGraphPoses();
}

PoseGraphPose PolygonExplorer::getPoseGraphPose(unsigned int pose_id) const {
  return poseGraph_.getPoseGraphPose(pose_id);
}
