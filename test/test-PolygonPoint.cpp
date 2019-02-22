#include "PolygonPoint.h"
#include "gtest/gtest.h"

namespace {

// The fixture for testing class Polygon.
class PolygonPointTest : public ::testing::Test {
 protected:
  PolygonPointTest() = default;
};

// Tests that a Point is constructed correctly
TEST_F(PolygonPointTest, CreatePoint1) {
  PolygonPoint point(2.0, 5.0, PointType::MAX_RANGE);

  ASSERT_EQ(point.getX(), 2.0) << "X value wrong";
  ASSERT_EQ(point.getY(), 5.0) << "Y value wrong";

  ASSERT_EQ(point.getPointType(), PointType::MAX_RANGE);
}

// Tests that a Point is constructed correctly
TEST_F(PolygonPointTest, CreatePoint2) {
  PolygonPoint point(2.0, 5.0, PointType::OBSTACLE);

  ASSERT_EQ(point.getX(), 2.0) << "X value wrong";
  ASSERT_EQ(point.getY(), 5.0) << "Y value wrong";

  ASSERT_EQ(point.getPointType(), PointType::OBSTACLE);
}

TEST_F(PolygonPointTest, CompareUnequalPoints1) {
  PolygonPoint point_1(2.0, 5.0, PointType::OBSTACLE);
  PolygonPoint point_2(1.0, 4.0, PointType::OBSTACLE);

  ASSERT_NE(point_1, point_2) << "Points are equal";
}

TEST_F(PolygonPointTest, CompareUnequalPoints2) {
  PolygonPoint point_1(2.0, 5.0, PointType::OBSTACLE);
  PolygonPoint point_2(1.0, 4.0, PointType::OBSTACLE);

  ASSERT_TRUE(point_1 != point_2) << "Points are equal";
}

TEST_F(PolygonPointTest, CompareUnequalPoints3) {
  PolygonPoint point_1(2.0, 5.0, PointType::OBSTACLE);
  PolygonPoint point_2(1.0, 4.0, PointType::OBSTACLE);

  ASSERT_FALSE(point_1 == point_2) << "Points are equal";
}

TEST_F(PolygonPointTest, CompareUnequalPoints4) {
  PolygonPoint point_1(2.0, 5.0, PointType::OBSTACLE);
  PolygonPoint point_2(2.0, 5.0, PointType::UNKNOWN);

  ASSERT_NE(point_1, point_2) << "Points are equal";
}

TEST_F(PolygonPointTest, CompareEqualPoints1) {
  PolygonPoint point_1(2.0, 5.0, PointType::OBSTACLE);
  PolygonPoint point_2(2.0, 5.0, PointType::OBSTACLE);

  ASSERT_EQ(point_1, point_2) << "Points are not equal";
}

TEST_F(PolygonPointTest, CompareEqualPoints2) {
  PolygonPoint point_1(2.0, 5.0, PointType::OBSTACLE);
  PolygonPoint point_2(2.0, 5.0, PointType::OBSTACLE);

  ASSERT_TRUE(point_1 == point_2) << "Points are not equal";
}

TEST_F(PolygonPointTest, CompareEqualPoints3) {
  PolygonPoint point_1(2.0, 5.0, PointType::OBSTACLE);
  PolygonPoint point_2(2.0, 5.0, PointType::OBSTACLE);

  ASSERT_FALSE(point_1 != point_2) << "Points are not equal";
}

}  // namespace
