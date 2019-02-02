#include "Point.h"
#include "gtest/gtest.h"

namespace {

// The fixture for testing class Polygon.
class PointTest : public ::testing::Test {
 protected:
  PointTest() = default;
};

// Tests that a Point is constructed correctly
TEST_F(PointTest, CreatePoint) {
  Point point(2.0, 5.0);

  ASSERT_EQ(point.getX(), 2.0) << "X value wrong";
  ASSERT_EQ(point.getY(), 5.0) << "Y value wrong";
}

TEST_F(PointTest, CompareUnequalPoints) {
  Point point_1(2.0, 5.0);
  Point point_2(1.0, 4.0);

  ASSERT_NE(point_1, point_2) << "Points are equal";
}

TEST_F(PointTest, CompareEqualPoints) {
  Point point_1(2.0, 5.0);
  Point point_2(2.0, 5.0);

  ASSERT_EQ(point_1, point_2) << "Points are not equal";
}


}  // namespace
