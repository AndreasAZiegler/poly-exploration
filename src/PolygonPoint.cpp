/* Point representation for polygons
 */

#include "PolygonPoint.h"

PolygonPoint::PolygonPoint(double x, double y, bool max_sensor_range)
    : Point(x, y),
      maxSensorRange_(max_sensor_range) {
} /* -----  end of method PolygonPoint::PolygonPoint  (constructor)  ----- */

