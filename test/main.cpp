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

  PolyTest() {
    // You can do set-up work for each test here.
  }

  ~PolyTest() override {
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
  points.emplace_back({0.0, 0.0});
  points.emplace_back({10.0, 10.0});
  points.emplace_back({10.0, 0, 0});

  Polygon polygon(points);

  auto return_points = polygon.getPoints();

  ASSERT_EQ(points.size(), return_points.size())
      << "Vectors points and return_points are of unequal length";

  for (int i = 0; i < return_points.size(); ++i) {
    EXPECT_EQ(points[i], return_points[i])
        << "Vectors points and return_points differ at index " << i;
  }
}

TEST_F(PolygonTest, CreatePolygonUnion) {
  std::vector<Point> first_polygon_points;
  first_polygon_points.emplace_back({0.0, 0.0});
  first_polygon_points.emplace_back({0.0, 10.0});
  first_polygon_points.emplace_back({10.0, 10.0});
  first_polygon_points.emplace_back({10.0, 0, 0});

  Polygon first_polygon(first_polygon_points);

  std::vector<Point> second_polygon_points;
  second_polygon_points.emplace_back({5.0, 5.0});
  second_polygon_points.emplace_back({15.0, 5.0});
  second_polygon_points.emplace_back({15.0, 15.0});
  second_polygon_points.emplace_back({15.0, 5.0});

  Polygon second_polygon(second_polygon_points);

  auto return_first_polygon_points = first_polygon.getPoints();

  ASSERT_EQ(points.size(), return_points.size())
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

  ASSERT_EQ((first_polygon_points.size() + second_polygon_points.size()), union_polygon.size())
      << "Vectors points and return_points are of unequal length";

}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
