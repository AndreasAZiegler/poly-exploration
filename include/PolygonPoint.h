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

  void setInFreeSpace(bool in_free_space) { inFreeSpace_ = in_free_space; }

  PointType getPointType() const { return pointType_; }

  bool isInFreeSpace() const { return inFreeSpace_; }

 private:
  PointType pointType_;
  bool inFreeSpace_;
}; /* -----  end of class PolygonPoint  ----- */

