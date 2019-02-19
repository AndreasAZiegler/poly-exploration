/* Pose graph representation
 */

#pragma once

#include <vector>
#include "Polygon.h"
#include "Geometry.h"
#include "PoseGraphPose.h"

class PoseGraph {
 public:
  PoseGraph();

  void addPose(Polygon& polygon, Pose transformation);

  PoseGraphPose& getPoseGraphPose(void);

  PoseGraphPose& getPoseGraphPose(const unsigned int id);

 private:
  unsigned int currentPoseGraphPoseId_;

  std::vector<PoseGraphPose> poseGraphPoses_;
}; /* -----  end of class PoseGraph  ----- */

