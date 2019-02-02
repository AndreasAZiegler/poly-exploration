#include <vector>

#include "gtest/gtest.h"
#include "Polygon.h"

namespace {

// The fixture for testing class Polygon.
class PolygonTest : public ::testing::Test {
 protected:
  PolygonTest() = default;
};

// Tests that Polygons are constructed correctly
TEST_F(PolygonTest, CreatePolygonWithPoints) {
  std::vector<Point> points;
  points.emplace_back(Point(0.0, 0.0));
  points.emplace_back(Point(10.0, 10.0));
  points.emplace_back(Point(10.0, 0));
  points.emplace_back(Point(0.0, 0.0));

  Polygon polygon(points);

  auto return_points = polygon.getPoints();

  ASSERT_EQ(points.size(), return_points.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_points.size(); ++i) {
    EXPECT_EQ(points[i], return_points[i])
        << "Vectors points and return_points differ at index " << i;
  }
}

TEST_F(PolygonTest, CreatePolygonWithCopyConstructor) {
  std::vector<Point> points;
  points.emplace_back(Point(0.0, 0.0));
  points.emplace_back(Point(10.0, 10.0));
  points.emplace_back(Point(10.0, 0));
  points.emplace_back(Point(0.0, 0.0));

  Polygon polygon_1(points);

  Polygon polygon_2(polygon_1);

  auto return_points = polygon_2.getPoints();

  ASSERT_EQ(points.size(), return_points.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_points.size(); ++i) {
    EXPECT_EQ(points[i], return_points[i])
        << "Vectors points and return_points differ at index " << i;
  }
}

TEST_F(PolygonTest, CreatePolygonWithCopy) {
  std::vector<Point> points;
  points.emplace_back(Point(0.0, 0.0));
  points.emplace_back(Point(10.0, 10.0));
  points.emplace_back(Point(10.0, 0));
  points.emplace_back(Point(0.0, 0.0));

  Polygon polygon_1(points);

  Polygon polygon_2 = polygon_1;

  auto return_points = polygon_2.getPoints();

  ASSERT_EQ(points.size(), return_points.size())
      << "Vectors points and return_points are of unequal length";

  for (unsigned int i = 0; i < return_points.size(); ++i) {
    EXPECT_EQ(points[i], return_points[i])
        << "Vectors points and return_points differ at index " << i;
  }
}

TEST_F(PolygonTest, CreatePolygonUnion1) {
  std::vector<Point> first_polygon_points;
  first_polygon_points.emplace_back(Point(0.0, 0.0));
  first_polygon_points.emplace_back(Point(0.0, 10.0));
  first_polygon_points.emplace_back(Point(10.0, 10.0));
  first_polygon_points.emplace_back(Point(10.0, 0));
  first_polygon_points.emplace_back(Point(0.0, 0.0));

  Polygon first_polygon(first_polygon_points);
  first_polygon.plot("first_polygon.svg");

  std::vector<Point> second_polygon_points;
  second_polygon_points.emplace_back(Point(5.0, 5.0));
  second_polygon_points.emplace_back(Point(5.0, 15.0));
  second_polygon_points.emplace_back(Point(15.0, 15.0));
  second_polygon_points.emplace_back(Point(15.0, 5.0));
  second_polygon_points.emplace_back(Point(5.0, 5.0));

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
}

TEST_F(PolygonTest, GetNumberOfIntersectionsOverlappingPolygons) {
  std::vector<Point> first_polygon_points;
  first_polygon_points.emplace_back(Point(0.0, 0.0));
  first_polygon_points.emplace_back(Point(0.0, 10.0));
  first_polygon_points.emplace_back(Point(10.0, 10.0));
  first_polygon_points.emplace_back(Point(10.0, 0));
  first_polygon_points.emplace_back(Point(0.0, 0.0));

  Polygon first_polygon(first_polygon_points);
  first_polygon.plot("first_polygon.svg");

  std::vector<Point> second_polygon_points;
  second_polygon_points.emplace_back(Point(5.0, 5.0));
  second_polygon_points.emplace_back(Point(5.0, 15.0));
  second_polygon_points.emplace_back(Point(15.0, 15.0));
  second_polygon_points.emplace_back(Point(15.0, 5.0));
  second_polygon_points.emplace_back(Point(5.0, 5.0));

  Polygon second_polygon(second_polygon_points);

  auto number_of_intersections = first_polygon.getNumberOfIntersections(second_polygon);

  ASSERT_EQ(number_of_intersections, 4) << "Wrong number of intersections";
}

TEST_F(PolygonTest, GetNumberOfIntersectionsNonOverlappingPolygons) {
  std::vector<Point> first_polygon_points;
  first_polygon_points.emplace_back(Point(0.0, 0.0));
  first_polygon_points.emplace_back(Point(0.0, 10.0));
  first_polygon_points.emplace_back(Point(10.0, 10.0));
  first_polygon_points.emplace_back(Point(10.0, 0));
  first_polygon_points.emplace_back(Point(0.0, 0.0));

  Polygon first_polygon(first_polygon_points);

  std::vector<Point> second_polygon_points;
  second_polygon_points.emplace_back(Point(15.0, 15.0));
  second_polygon_points.emplace_back(Point(5.0, 25.0));
  second_polygon_points.emplace_back(Point(25.0, 25.0));
  second_polygon_points.emplace_back(Point(25.0, 5.0));
  second_polygon_points.emplace_back(Point(15.0, 15.0));

  Polygon second_polygon(second_polygon_points);

  auto number_of_intersections = first_polygon.getNumberOfIntersections(second_polygon);

  ASSERT_EQ(number_of_intersections, 0) << "Wrong number of intersections";
}

TEST_F(PolygonTest, CreatePolygonUnion2) {
  std::vector<Point> first_polygon_points;
  first_polygon_points.emplace_back(Point(2.0, 1.3));
  first_polygon_points.emplace_back(Point(2.4, 1.7));
  first_polygon_points.emplace_back(Point(2.8, 1.8));
  first_polygon_points.emplace_back(Point(3.4, 1.2));
  first_polygon_points.emplace_back(Point(3.7, 1.6));
  first_polygon_points.emplace_back(Point(3.4, 2.0));
  first_polygon_points.emplace_back(Point(4.1, 3.0));
  first_polygon_points.emplace_back(Point(5.3, 2.6));
  first_polygon_points.emplace_back(Point(5.4, 1.2));
  first_polygon_points.emplace_back(Point(4.9, 0.8));
  first_polygon_points.emplace_back(Point(2.9, 0.7));
  first_polygon_points.emplace_back(Point(2.0, 1.3));

  Polygon first_polygon(first_polygon_points);

  std::vector<Point> second_polygon_points;
  second_polygon_points.emplace_back(Point(4.0, -0.5));
  second_polygon_points.emplace_back(Point(3.5, 1.0));
  second_polygon_points.emplace_back(Point(2.0, 1.5));
  second_polygon_points.emplace_back(Point(3.5, 2.0));
  second_polygon_points.emplace_back(Point(4.0, 3.5));
  second_polygon_points.emplace_back(Point(4.5, 2.0));
  second_polygon_points.emplace_back(Point(6.0, 1.5));
  second_polygon_points.emplace_back(Point(4.5, 1.0));
  second_polygon_points.emplace_back(Point(4.0, -0.5));

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
}

}  // namespace
