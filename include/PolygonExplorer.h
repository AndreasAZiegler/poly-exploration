/* Polygon explorer (main) class representation
 */

#pragma once

#include <vector>
#include "PoseGraph.h"
#include "eventpp/callbacklist.h"

class PolygonExplorer {
 public:
  PolygonExplorer();

  void setCallBack(std::function<void(const PoseGraph)> callback_function);

  void addPose(const Pose& current_pose_to_previews_pose_transformation,
               const Polygon& polygon);

  std::vector<PoseGraphPose> getPoseGraphPoses() const;

  PoseGraphPose getPoseGraphPose(unsigned int pose_id) const;

 private:
  PoseGraph poseGraph_;

  eventpp::CallbackList<void(const PoseGraph)> callBackList_;

}; /* -----  end of class PolygonExplorer  ----- */

