/* Point representation for polygons
 */

#pragma once

#include "Point.h"

enum class PointType { UNKNOWN, MAX_RANGE, OBSTACLE };

class PolygonPoint : public Point {
 public:
  PolygonPoint(double x, double y, PointType point_type);

  bool operator==(const PolygonPoint& point);

  friend bool operator==(const PolygonPoint& lhs,
                                 const PolygonPoint& rhs);

  virtual bool operator!=(const PolygonPoint& point);

  friend bool operator!=(const PolygonPoint& lhs,
                         const PolygonPoint& rhs);

  PointType getPointType() const;

 private:
  PointType pointType_;
  bool inFreeSpace_;
}; /* -----  end of class PolygonPoint  ----- */

