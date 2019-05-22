/* Polygon explorer (main) class representation
 */

#pragma once

#include <vector>
#include "PolygonExplorerInterface.h"
#include "PoseGraph.h"
#include "eventpp/callbacklist.h"

class PolygonExplorer {
 public:
  PolygonExplorer() = default;

  void setCallBack(PolygonExplorerInterface* polygon_explorer_node);

  void addPose(const Pose& current_pose_to_previews_pose_transformation,
               const Polygon& polygon, TimeStamp time_stamp);

  std::vector<PoseGraphPose> getPoseGraphPoses() const;

  PoseGraphPose getPoseGraphPose(unsigned int pose_id) const;

 private:
  PoseGraph poseGraph_;

  eventpp::CallbackList<void(PoseGraph, TimeStamp)> callBackList_;

}; /* -----  end of class PolygonExplorer  ----- */

