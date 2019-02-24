/* Pose graph representation
 */

#pragma once

#include <vector>
#include "Geometry.h"
#include "Polygon.h"
#include "PoseGraphPose.h"

class PoseGraph {
 public:
  PoseGraph();

  void addPose(Polygon& polygon, Pose transformation);

  void connectTwoPoses(unsigned int pose_id_1, unsigned int pose_id_2,
                       Pose transformation);

  PoseGraphPose& getPoseGraphPose(void);

  PoseGraphPose& getPoseGraphPose(const unsigned int id);

 private:
  void consolidatePolygon(unsigned int pose_graph_pose_id);

  unsigned int currentPoseGraphPoseId_;

  std::vector<PoseGraphPose> poseGraphPoses_;
}; /* -----  end of class PoseGraph  ----- */

