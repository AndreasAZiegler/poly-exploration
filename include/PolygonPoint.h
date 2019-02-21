/* Point representation for polygons
 */

#pragma once

#include "Point.h"

enum class PointType { UNKNOWN, MAX_RANGE, OBSTACLE };

class PolygonPoint : public Point {
 public:
  PolygonPoint(double x, double y, PointType point_type);

  void setInFreeSpace(bool in_free_space) { inFreeSpace_ = in_free_space; }

  PointType getPointType() { return pointType_; }

  bool isInFreeSpace() { return inFreeSpace_; }

 private:
  PointType pointType_;
  bool inFreeSpace_;
}; /* -----  end of class PolygonPoint  ----- */

