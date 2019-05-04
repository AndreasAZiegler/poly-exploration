#include <glog/logging.h>
#include "gtest/gtest.h"
#include "poly_exploration/PolygonExplorer.h"
#include "poly_exploration/PolygonExplorerInterface.h"

class PolygonExplorerNode : public PolygonExplorerInterface {
 public:
  PolygonExplorerNode() { polygonExplorer_.setCallBack(this); }

  virtual void updateVisualizationCallback(
      const PoseGraph pose_graph, const TimeStamp time_stamp) override {
    CHECK(!pose_graph.getPoseGraphPoses().empty())
        << "There should be at least one pose in the pose graph!";

    std::cout << "Update visualization." << std::endl;

    std::map<unsigned int, Pose> pose_graph_transformations;

    std::queue<unsigned int> pose_ids_to_check;
    std::set<unsigned int> pose_ids_checked;

    pose_ids_to_check.emplace(0);
    pose_graph_transformations[0] = Pose();

    while (!pose_ids_to_check.empty()) {
      auto id_to_check = pose_ids_to_check.front();
      std::cout << "id_to_check: " << id_to_check << std::endl;
      pose_ids_to_check.pop();
      pose_ids_checked.emplace(id_to_check);

      auto adjacent_ids =
          pose_graph.getPoseGraphPose(id_to_check).getAdjacentPosesId();
      auto adjacent_ids_pose =
          pose_graph.getPoseGraphPose(id_to_check).getAdjacentPoses();

      for (const auto& adjacent_id : adjacent_ids) {
        auto check_to_adjacent_transformation = adjacent_ids_pose[adjacent_id];
        auto origin_to_adjacent_transformation =
            pose_graph_transformations[id_to_check] *
            check_to_adjacent_transformation;

        if (0 == pose_ids_checked.count(adjacent_id)) {
          pose_ids_to_check.emplace(adjacent_id);
        }
        pose_graph_transformations[adjacent_id] =
            origin_to_adjacent_transformation;
        std::cout << "Origin to " << id_to_check
                  << " transformation: " << std::endl
                  << pose_graph_transformations[id_to_check] << std::endl;
        std::cout << "pose " << id_to_check << " to pose " << adjacent_id
                  << " transformation: " << std::endl
                  << check_to_adjacent_transformation << std::endl;
        std::cout << "Origin to " << adjacent_id
                  << " transformation: " << std::endl
                  << origin_to_adjacent_transformation << std::endl;

        auto first_point_x =
            pose_graph_transformations[id_to_check].getPosition().x();
        auto first_point_y =
            pose_graph_transformations[id_to_check].getPosition().y();

        auto second_point_x =
            origin_to_adjacent_transformation.getPosition().x();
        auto second_point_y =
            origin_to_adjacent_transformation.getPosition().y();
      }
    }

    std::cout << "Updated visualization." << std::endl;
  }

  PolygonExplorer polygonExplorer_;

 private:
  Pose previousPose_;
};

namespace {

// The fixture for testing class Polygon.
class PolygonExplorerInterfaceTest : public ::testing::Test {
 protected:
  PolygonExplorerInterfaceTest() = default;
};

// Tests that PoseGraphPose is constructed correctly
TEST_F(PolygonExplorerInterfaceTest, CreatePolygonExplorerInterface) {
  PolygonExplorerNode polygon_explorer_node;

  double angle_min = -3.141 / 2;
  double angle_increment = 3.141 / 10;
  std::vector<PolygonPoint> polygon_points;
  for (unsigned int i = 0; i < 11; ++i) {
    double theta = angle_min + i * angle_increment;
    double x = 16.9 * cos(theta);
    double y = 16.9 * sin(theta);

    PointType point_type;
    if (16.9 >= 17.0) {
      point_type = PointType::MAX_RANGE;
    } else {
      point_type = PointType::OBSTACLE;
    }

    polygon_points.emplace_back(x, y, point_type);
  }

  // Polygon has to be closed (first and last point have to be the same)
  double theta = angle_min + 0 * angle_increment;
  double x = 16.9 * cos(theta);
  double y = 16.9 * sin(theta);
  PointType point_type;
  if (16.9 >= 17.0) {
    point_type = PointType::MAX_RANGE;
  } else {
    point_type = PointType::OBSTACLE;
  }

  polygon_points.emplace_back(x, y, point_type);

  Polygon polygon(polygon_points);

  Pose transformation_previous_pose_current_pose(Position(1.0, 0.0, 0.0),
                                                 Rotation());

  TimeStamp time_stamp = {0, 0};
  polygon_explorer_node.polygonExplorer_.addPose(
      transformation_previous_pose_current_pose, polygon, time_stamp);

  polygon_explorer_node.polygonExplorer_.addPose(
      transformation_previous_pose_current_pose, polygon, time_stamp);
}
}  // namespace
