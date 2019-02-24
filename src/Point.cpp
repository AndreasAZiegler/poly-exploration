#include "Point.h"

Point::Point(double x, double y) {
  x_ = x;
  y_ = y;
}

bool Point::operator==(const Point& point) {
  return (fabs(this->x_ - point.x_) < 1e-4) &&
         (fabs(this->y_ - point.y_) < 1e-4);
}

bool Point::operator!=(const Point& point) {
  return (fabs(this->x_ - point.x_) > 1e-4) ||
         (fabs(this->y_ - point.y_) > 1e-4);
}

bool operator==(const Point& lhs, const Point& rhs) {
  return (fabs(lhs.x_ - rhs.x_) < 1e-4) && (fabs(lhs.y_ - rhs.y_) < 1e-4);
}

bool operator!=(const Point& lhs, const Point& rhs) {
  return (fabs(lhs.x_ - rhs.x_) > 1e-4) || (fabs(lhs.y_ - rhs.y_) > 1e-4);
}

double Point::getX() const { return x_; };

double Point::getY() const { return y_; };
