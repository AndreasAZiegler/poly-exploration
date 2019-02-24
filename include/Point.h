/* Point representation
 */

#pragma once

#include <cmath>

class Point {
 public:
  Point(double x, double y);

  bool operator==(const Point& point);

  friend bool operator==(const Point& lhs, const Point& rhs);

  bool operator!=(const Point& point);

  friend bool operator!=(const Point& lhs, const Point& rhs);

  double getX() const;

  double getY() const;

 protected:
  double x_;
  double y_;
};
