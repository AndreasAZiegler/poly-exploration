/* Polygon representation based on boost::Geometry.
 */

#include "Polygon.h"
#include "Point.h"

Polygon::Polygon(std::vector<Point> points) {
  for (const auto& point : points) {
    points_.emplace_back(point);

    auto boost_point = BoostPoint(point.x_, point.y_);
    boost::geometry::append(polygon_.outer(), boost_point);
  }
} /* -----  end of method Polygon::Polygon  (constructor)  ----- */

Polygon::Polygon(const Polygon& other) {
} /* -----  end of method Polygon::Polygon  (copy constructor)  ----- */

Polygon::~Polygon() {
} /* -----  end of method Polygon::~Polygon  (destructor)  ----- */

Polygon& Polygon::operator=(const Polygon& other) {
  if (this != &other) {
  }

  return *this;
} /* -----  end of method Polygon::operator =  (assignment operator)  ----- */

Polygon Polygon::buildUnion(const Polygon& polygon) const {
  // Boost output polygon
  std::vector<BoostPolygon> output_polygons;

  // Union
  boost::geometry::union_(polygon_, polygon.polygon_, output_polygons);

  // Reference to the output polygon
  const auto& output_polygon = output_polygons[0];

  return std::move(getPolygonFromBoostPolygon(output_polygon));
}

Polygon Polygon::getPolygonFromBoostPolygon(const BoostPolygon polygon) const {
  // Reference to the points of the input polygon
  const auto& polygon_points = polygon.outer();

  // Points to create a new Polygon
  std::vector<Point> points;

  for (std::vector<BoostPoint>::size_type i = 0; i < polygon_points.size();
       ++i) {
    points.emplace_back(Point(boost::geometry::get<0>(polygon_points[i]),
                              boost::geometry::get<1>(polygon_points[i])));
  }

  return std::move(Polygon(points));
}

std::vector<Point>& Polygon::getPoints(void) { return points_; }
