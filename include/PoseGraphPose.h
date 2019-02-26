/* Pose graph pose representation
 */

#pragma once

#include <map>
#include <memory>
#include <tuple>
#include <vector>
#include "Geometry.h"
#include "Polygon.h"

class PoseGraphPose {
 public:
  PoseGraphPose(const Polygon& polygon, unsigned int pose_graph_pose_id);

  void addAdjacentPose(unsigned int pose_graph_pose_id,
                       const Pose& transformation);

  Polygon& getPolygon();

  std::map<unsigned int, Pose> getAdjacentPoses();

  std::vector<unsigned int> getAdjacentPosesId();

  int getId();

 private:
  int id_;

  Polygon polygon_;

  std::map<unsigned int, Pose> adjacentPoses_;
}; /* -----  end of class PoseGraphPose  ----- */

