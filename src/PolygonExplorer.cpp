/* Polygon explorer (main) class representation
 */

#include "poly_exploration/PolygonExplorer.h"

PolygonExplorer::PolygonExplorer() {} /* -----  end of method
                                         PolygonExplorer::PolygonExplorer
                                         (constructor)  ----- */

void PolygonExplorer::setCallBack(
    PolygonExplorerInterface* polygon_explorer_node) {
  callBackList_.append(
      std::bind(&PolygonExplorerInterface::updateVisualizationCallback,
                polygon_explorer_node, std::placeholders::_1));
  /*
  callBackList_.append([polygon_explorer_node](PoseGraph pose_graph) {
    polygon_explorer_node->updateVisualizationCallback(pose_graph);
  });
  */
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
