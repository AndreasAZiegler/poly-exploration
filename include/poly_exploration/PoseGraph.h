/* Pose graph representation
 */

#pragma once

#include <gtest/gtest_prod.h>
#include <queue>
#include <set>
#include <stack>
#include <tuple>
#include <vector>
#include "Geometry.h"
#include "Polygon.h"
#include "PoseGraphPose.h"

class PoseGraph {
 public:
  PoseGraph();

  void addPose(const Pose& current_pose_to_previews_pose_transformation,
               const Polygon& polygon);

  void connectTwoPoses(unsigned int pose_id_1, unsigned int pose_id_2,
                       Pose transformation);

  std::vector<PoseGraphPose> getPoseGraphPoses() const;

  PoseGraphPose getPoseGraphPose() const;

  PoseGraphPose getPoseGraphPose(unsigned int id) const;

 private:
  void consolidatePolygon(unsigned int pose_graph_pose_id);

  unsigned int currentPoseGraphPoseId_;

  std::vector<PoseGraphPose> poseGraphPoses_;
}; /* -----  end of class PoseGraph  ----- */

