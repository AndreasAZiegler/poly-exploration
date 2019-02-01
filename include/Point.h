/* Point representation
 */

#pragma once

#include <cmath>

struct Point {
  Point(double x, double y) {
    x_ = x;
    y_ = y;
  }

  inline bool operator==(const Point& point) {
    return (fabs(this->x_ - point.x_) < 1e-4) &&
           (fabs(this->y_ - point.y_) < 1e-4);
  }

  inline friend bool operator==(const Point& lhs, const Point& rhs) {
    return (fabs(lhs.x_ - rhs.x_) < 1e-4) && (fabs(lhs.y_ - rhs.y_) < 1e-4);
  }

  inline bool operator!=(const Point& point) {
    return (fabs(this->x_ - point.x_) > 1e-4) ||
           (fabs(this->y_ - point.y_) > 1e-4);
  }

  inline friend bool operator!=(const Point& lhs, const Point& rhs) {
    return (fabs(lhs.x_ - rhs.x_) > 1e-4) || (fabs(lhs.y_ - rhs.y_) > 1e-4);
  }

  double x_;
  double y_;
};
