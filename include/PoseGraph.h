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

  void addPose(const Polygon& polygon, Pose transformation);

  void connectTwoPoses(unsigned int pose_id_1, unsigned int pose_id_2,
                       Pose transformation);

  std::vector<PoseGraphPose> getPoseGraphPoses();

  PoseGraphPose& getPoseGraphPose();

  PoseGraphPose& getPoseGraphPose(unsigned int id);

 private:
  void consolidatePolygon(unsigned int pose_graph_pose_id);

  unsigned int currentPoseGraphPoseId_;

  std::vector<PoseGraphPose> poseGraphPoses_;

 public:
  FRIEND_TEST(PoseGraphTest, ConsolidatePolygon);
  FRIEND_TEST(PoseGraphTest, GetIntersectedPolygonOwners1);
}; /* -----  end of class PoseGraph  ----- */

