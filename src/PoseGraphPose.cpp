/* Pose graph pose representation
 */

#include "PoseGraphPose.cpp"

PoseGraphPose::PoseGraphPose(Polygon polygon,
                             std::vector<PoseGraphPose> adjacent_poses,
                             Pose pose) {
  polygon_ = polygon;
  adjacentPoses_ = adjacent_poses;
  pose_ = pose;
} /* -----  end of method PoseGraphPose::PoseGraphPose  (constructor)  ----- */

