/*
 */

#include "Polygon.h"
#include "gtest/gtest.h"

namespace {

// The fixture for testing class Polygon.
class PolygonTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  PolygonTest() {
    // You can do set-up work for each test here.
  }

  ~PolygonTest() override {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  void TearDown() override {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for Foo.
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

  for (int i = 0; i < return_points.size(); ++i) {
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
  /*
  first_polygon.print();
  */
  first_polygon.plot("first_polygon.svg");

  std::vector<Point> second_polygon_points;
  second_polygon_points.emplace_back(Point(5.0, 5.0));
  second_polygon_points.emplace_back(Point(5.0, 15.0));
  second_polygon_points.emplace_back(Point(15.0, 15.0));
  second_polygon_points.emplace_back(Point(15.0, 5.0));
  second_polygon_points.emplace_back(Point(5.0, 5.0));

  Polygon second_polygon(second_polygon_points);
  /*
  second_polygon.print();
  */
  second_polygon.plot("second_polygon.svg");

  auto return_first_polygon_points = first_polygon.getPoints();

  ASSERT_EQ(first_polygon_points.size(), return_first_polygon_points.size())
      << "Vectors points and return_points are of unequal length";

  for (int i = 0; i < return_first_polygon_points.size(); ++i) {
    EXPECT_EQ(first_polygon_points[i], return_first_polygon_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  auto return_second_polygon_points = second_polygon.getPoints();

  ASSERT_EQ(second_polygon_points.size(), return_second_polygon_points.size())
      << "Vectors points and return_points are of unequal length";

  for (int i = 0; i < return_second_polygon_points.size(); ++i) {
    EXPECT_EQ(second_polygon_points[i], return_second_polygon_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  auto union_polygon = first_polygon.buildUnion(second_polygon);
  /*
  union_polygon.print();
  */
  union_polygon.plot("union.svg");
  /*
  std::cout << "Number of intersections: "
            << first_polygon.getNumberOfIntersections(second_polygon)
            << std::endl;
  */

  ASSERT_LE(
      ((first_polygon_points.size() - 1) + (second_polygon_points.size() - 1)),
      (union_polygon.getPoints().size() - 1))
      << "Vectors points and return_points are of unequal length";
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
  //first_polygon.plot("firs_polygon.svg");

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
  //second_polygon.plot("second_polygon.svg");

  auto return_first_polygon_points = first_polygon.getPoints();

  ASSERT_EQ(first_polygon_points.size(), return_first_polygon_points.size())
      << "Vectors points and return_points are of unequal length";

  for (int i = 0; i < return_first_polygon_points.size(); ++i) {
    EXPECT_EQ(first_polygon_points[i], return_first_polygon_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  auto return_second_polygon_points = second_polygon.getPoints();

  ASSERT_EQ(second_polygon_points.size(), return_second_polygon_points.size())
      << "Vectors points and return_points are of unequal length";

  for (int i = 0; i < return_second_polygon_points.size(); ++i) {
    EXPECT_EQ(second_polygon_points[i], return_second_polygon_points[i])
        << "Vectors points and return_points differ at index " << i;
  }

  auto union_polygon = first_polygon.buildUnion(second_polygon);
  //union_polygon.plot("union.svg");

  auto num_intersections =
      first_polygon.getNumberOfIntersections(second_polygon);
  //std::cout << "Number of intersections: " << num_intersections << std::endl;

  ASSERT_LE(
      ((first_polygon_points.size() - 1) + (second_polygon_points.size() - 1)),
      (union_polygon.getPoints().size() - 1))
      << "Vectors points and return_points are of unequal length";
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
