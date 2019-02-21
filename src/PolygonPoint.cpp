/* Point representation for polygons
 */

#include "PolygonPoint.h"

PolygonPoint::PolygonPoint(double x, double y, PointType point_type)
    : Point(x, y),
      pointType_(point_type) {
} /* -----  end of method PolygonPoint::PolygonPoint  (constructor)  ----- */

