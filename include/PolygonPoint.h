/* Point representation for polygons
 */

#pragma once

#include "Point.h"

class PolygonPoint : public Point {
 public:
  PolygonPoint(double x, double y, bool max_sensor_range);

  void setInFreeSpace(bool in_free_space) { inFreeSpace_ = in_free_space; }

  bool measuredAtMaximumSensorRange() { return maxSensorRange_; }

  bool isInFreeSpace() { return inFreeSpace_; }

 private:
  bool maxSensorRange_;
  bool inFreeSpace_;
}; /* -----  end of class PolygonPoint  ----- */

