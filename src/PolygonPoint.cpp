/* Point representation for polygons
 */

#include "PolygonPoint.h"

PolygonPoint::PolygonPoint(double x, double y, PointType point_type)
    : Point(x, y),
      pointType_(point_type) {
} /* -----  end of method PolygonPoint::PolygonPoint  (constructor)  ----- */

bool PolygonPoint::operator==(const PolygonPoint& point) {
  return (fabs(this->x_ - point.x_) < 1e-4) &&
         (fabs(this->y_ - point.y_) < 1e-4) &&
         (this->pointType_ == point.pointType_);
}

bool PolygonPoint::operator!=(const PolygonPoint& point) {
  return (fabs(this->x_ - point.x_) > 1e-4) ||
         (fabs(this->y_ - point.y_) > 1e-4) ||
         (this->pointType_ != point.pointType_);
}

bool operator==(const PolygonPoint& lhs,
                              const PolygonPoint& rhs) {
  return (fabs(lhs.x_ - rhs.x_) < 1e-4) && (fabs(lhs.y_ - rhs.y_) < 1e-4) &&
         (lhs.pointType_ == rhs.pointType_);
}

bool operator!=(const PolygonPoint& lhs,
                              const PolygonPoint& rhs) {
  return (fabs(lhs.x_ - rhs.x_) > 1e-4) || (fabs(lhs.y_ - rhs.y_) > 1e-4) ||
         (lhs.pointType_ != rhs.pointType_);
}
