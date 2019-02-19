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
  PoseGraphPose(
      Polygon& polygon, unsigned int pose_graph_pose_id);

  void addAdjacentPose(unsigned int pose_graph_pose_id,
                       std::shared_ptr<PoseGraphPose> pose_graph_pose,
                       Pose& transformation);

  Polygon& getPolygon() { return polygon_; }

  std::map<unsigned int, std::tuple<std::shared_ptr<PoseGraphPose>, Pose>>
  getAdjacentPoses() {
    return adjacentPoses_;
  }

  int getId() { return id_; }

 private:
  int id_;

  Polygon polygon_;

  std::map<unsigned int, std::tuple<std::shared_ptr<PoseGraphPose>, Pose>>
      adjacentPoses_;
}; /* -----  end of class PoseGraphPose  ----- */

