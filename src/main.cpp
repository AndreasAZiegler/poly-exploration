/* Main file
 */

#include "Polygon.h"
#include "PolygonConsolidation.h"
#include <glog/logging.h>

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);  // init google logging

  PoseGraph pose_graph;

  std::vector<PolygonPoint> points;
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 10.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 7.5, PointType::OBSTACLE);
  points.emplace_back(10.0, 5.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 2.5, PointType::OBSTACLE);
  points.emplace_back(10.0, 0.0, PointType::OBSTACLE);
  points.emplace_back(10.0, -2.5, PointType::OBSTACLE);
  points.emplace_back(10.0, -5.0, PointType::OBSTACLE);
  points.emplace_back(10.0, -7.5, PointType::OBSTACLE);
  points.emplace_back(10.0, -10.0, PointType::OBSTACLE);
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon polygon(points);

  pose_graph.addPose(polygon, Pose());

  kindr::AngleAxisD angleAxis1(0 /*0.7853 45deg*/, Eigen::Vector3d::UnitZ());
  Pose transformation1(Position(4.0, 6.0, 0), Rotation(angleAxis1));

  pose_graph.addPose(polygon, transformation1);

  kindr::AngleAxisD angleAxis2(0 /*-0.7853 -45deg*/, Eigen::Vector3d::UnitZ());
  Pose transformation2(Position(4.0, -5.0, 0), Rotation(angleAxis2));

  pose_graph.addPose(polygon, transformation2);
}
