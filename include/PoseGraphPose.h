/* Pose graph pose representation
 */

#pragma once

#include <vector>
#include "Geometry.h"
#include "Polygon.h"

class PoseGraphPose {
 public:
  PoseGraphPose(Polygon polygon, std::vector<PoseGraphPose> adjacent_poses,
                Pose pose);

 private:
  Polygon polygon_;

  std::vector<PoseGraphPose> adjacentPoses_;

  Pose pose_
}; /* -----  end of class PoseGraphPose  ----- */

