#include <vector>

#include "Polygon.h"
#include "gtest/gtest.h"

namespace {

// The fixture for testing class Polygon.
class PolygonTest : public ::testing::Test {
 protected:
  PolygonTest() = default;
};

// Tests that Polygons are constructed correctly
TEST_F(PolygonTest, CreatePolygonWithPoints) {
  std::vector<PolygonPoint> points;
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 10.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 0, PointType::OBSTACLE);
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon polygon(points);

  auto return_points = polygon.getPoints();

  ASSERT_EQ(points.size(), return_points.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_points.size(); ++i) {
    EXPECT_EQ(points[i], return_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  ASSERT_EQ(polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";
}

TEST_F(PolygonTest, CreatePolygonWithCopyConstructor) {
  std::vector<PolygonPoint> points;
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 10.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 0, PointType::OBSTACLE);
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon polygon_1(points);

  Polygon polygon_2(polygon_1);

  auto return_points = polygon_2.getPoints();

  ASSERT_EQ(points.size(), return_points.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_points.size(); ++i) {
    EXPECT_EQ(points[i], return_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  ASSERT_EQ(polygon_1.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";

  ASSERT_EQ(polygon_2.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";
}

TEST_F(PolygonTest, CreatePolygonWithCopy) {
  std::vector<PolygonPoint> points;
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 10.0, PointType::OBSTACLE);
  points.emplace_back(10.0, 0, PointType::OBSTACLE);
  points.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon polygon_1(points);

  Polygon polygon_2 = polygon_1;

  auto return_points = polygon_2.getPoints();

  ASSERT_EQ(points.size(), return_points.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_points.size(); ++i) {
    EXPECT_EQ(points[i], return_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  ASSERT_EQ(polygon_1.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";

  ASSERT_EQ(polygon_2.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";
}

TEST_F(PolygonTest, CreatePolygonUnion1) {
  std::vector<PolygonPoint> first_polygon_points;
  first_polygon_points.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(0.0, 10.0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(10.0, 10.0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(10.0, 0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon first_polygon(first_polygon_points);

  std::vector<PolygonPoint> second_polygon_points;
  second_polygon_points.emplace_back(5.0, 5.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(5.0, 15.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(15.0, 15.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(15.0, 5.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(5.0, 5.0, PointType::OBSTACLE);

  Polygon second_polygon(second_polygon_points);

  auto return_first_polygon_points = first_polygon.getPoints();

  ASSERT_EQ(first_polygon_points.size(), return_first_polygon_points.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_first_polygon_points.size(); ++i) {
    EXPECT_EQ(first_polygon_points[i], return_first_polygon_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  auto return_second_polygon_points = second_polygon.getPoints();

  ASSERT_EQ(second_polygon_points.size(), return_second_polygon_points.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_second_polygon_points.size(); ++i) {
    EXPECT_EQ(second_polygon_points[i], return_second_polygon_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  auto union_polygon = first_polygon.buildUnion(second_polygon);

  ASSERT_LE(
      ((first_polygon_points.size() - 1) + (second_polygon_points.size() - 1)),
      (union_polygon.getPoints().size() - 1))
      << "Vectors points and return_points are of unequal length";

  ASSERT_EQ(first_polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";

  ASSERT_EQ(second_polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";

  ASSERT_EQ(union_polygon.isPolygonFromSensorMeasurements(), false)
      << "Polygon is built from sensor measurements";
}

TEST_F(PolygonTest, GetNumberOfIntersectionsOverlappingPolygons) {
  std::vector<PolygonPoint> first_polygon_points;
  first_polygon_points.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(0.0, 10.0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(10.0, 10.0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(10.0, 0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon first_polygon(first_polygon_points);

  std::vector<PolygonPoint> second_polygon_points;
  second_polygon_points.emplace_back(5.0, 5.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(5.0, 15.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(15.0, 15.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(15.0, 5.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(5.0, 5.0, PointType::OBSTACLE);

  Polygon second_polygon(second_polygon_points);

  auto number_of_intersections =
      first_polygon.getNumberOfIntersections(second_polygon);

  ASSERT_EQ(number_of_intersections, 4) << "Wrong number of intersections";

  ASSERT_EQ(first_polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";

  ASSERT_EQ(second_polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";
}

TEST_F(PolygonTest, GetNumberOfIntersectionsNonOverlappingPolygons) {
  std::vector<PolygonPoint> first_polygon_points;
  first_polygon_points.emplace_back(0.0, 0.0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(0.0, 10.0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(10.0, 10.0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(10.0, 0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(0.0, 0.0, PointType::OBSTACLE);

  Polygon first_polygon(first_polygon_points);

  std::vector<PolygonPoint> second_polygon_points;
  second_polygon_points.emplace_back(15.0, 15.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(5.0, 25.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(25.0, 25.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(25.0, 5.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(15.0, 15.0, PointType::OBSTACLE);

  Polygon second_polygon(second_polygon_points);

  auto number_of_intersections =
      first_polygon.getNumberOfIntersections(second_polygon);

  ASSERT_EQ(number_of_intersections, 0) << "Wrong number of intersections";

  ASSERT_EQ(first_polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";

  ASSERT_EQ(second_polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";
}

TEST_F(PolygonTest, CreatePolygonUnion2) {
  std::vector<PolygonPoint> first_polygon_points;
  first_polygon_points.emplace_back(2.0, 1.3, PointType::OBSTACLE);
  first_polygon_points.emplace_back(2.4, 1.7, PointType::OBSTACLE);
  first_polygon_points.emplace_back(2.8, 1.8, PointType::OBSTACLE);
  first_polygon_points.emplace_back(3.4, 1.2, PointType::OBSTACLE);
  first_polygon_points.emplace_back(3.7, 1.6, PointType::OBSTACLE);
  first_polygon_points.emplace_back(3.4, 2.0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(4.1, 3.0, PointType::OBSTACLE);
  first_polygon_points.emplace_back(5.3, 2.6, PointType::OBSTACLE);
  first_polygon_points.emplace_back(5.4, 1.2, PointType::OBSTACLE);
  first_polygon_points.emplace_back(4.9, 0.8, PointType::OBSTACLE);
  first_polygon_points.emplace_back(2.9, 0.7, PointType::OBSTACLE);
  first_polygon_points.emplace_back(2.0, 1.3, PointType::OBSTACLE);

  Polygon first_polygon(first_polygon_points);

  std::vector<PolygonPoint> second_polygon_points;
  second_polygon_points.emplace_back(4.0, -0.5, PointType::OBSTACLE);
  second_polygon_points.emplace_back(3.5, 1.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(2.0, 1.5, PointType::OBSTACLE);
  second_polygon_points.emplace_back(3.5, 2.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(4.0, 3.5, PointType::OBSTACLE);
  second_polygon_points.emplace_back(4.5, 2.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(6.0, 1.5, PointType::OBSTACLE);
  second_polygon_points.emplace_back(4.5, 1.0, PointType::OBSTACLE);
  second_polygon_points.emplace_back(4.0, -0.5, PointType::OBSTACLE);

  Polygon second_polygon(second_polygon_points);

  auto return_first_polygon_points = first_polygon.getPoints();

  ASSERT_EQ(first_polygon_points.size(), return_first_polygon_points.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_first_polygon_points.size(); ++i) {
    EXPECT_EQ(first_polygon_points[i], return_first_polygon_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  auto return_second_polygon_points = second_polygon.getPoints();

  ASSERT_EQ(second_polygon_points.size(), return_second_polygon_points.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_second_polygon_points.size(); ++i) {
    EXPECT_EQ(second_polygon_points[i], return_second_polygon_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  auto union_polygon = first_polygon.buildUnion(second_polygon);

  ASSERT_LE(
      ((first_polygon_points.size() - 1) + (second_polygon_points.size() - 1)),
      (union_polygon.getPoints().size() - 1))
      << "Vectors points and return_points are of unequal length";

  ASSERT_EQ(first_polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";

  ASSERT_EQ(second_polygon.isPolygonFromSensorMeasurements(), true)
      << "Polygon is not built from sensor measurements";

  ASSERT_EQ(union_polygon.isPolygonFromSensorMeasurements(), false)
      << "Polygon is built from sensor measurements";
}

}  // namespace
