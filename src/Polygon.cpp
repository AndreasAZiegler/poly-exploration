/* Polygon representation based on boost::Geometry.
 */

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "Polygon.h"

Polygon::Polygon(const std::vector<PolygonPoint>& points,
                 const std::vector<bool>& maximum_ranges)
    : polygonFromSensorMeasurements(true) {
  if (points[0] != points.back()) {
    throw std::invalid_argument(
        "First and last point of polygon have to be the same (Polygon should "
        "be closed)");
  }

  if (points.size() != maximum_ranges.size()) {
    throw std::invalid_argument(
        "points and max ranges have not the same size.");
  }

  for (const auto& point : points) {
    points_.emplace_back(point);

    auto boost_point = BoostPoint(point.getX(), point.getY());
    boost::geometry::append(polygon_.outer(), boost_point);
  }

  boost::geometry::correct(polygon_);

  maximum_ranges_ = maximum_ranges;
} /* -----  end of method Polygon::Polygon (constructor)  ----- */

Polygon::Polygon(const std::vector<Point>& points)
    : polygonFromSensorMeasurements(false) {
  if (points[0] != points.back()) {
    throw std::invalid_argument(
        "First and last point of polygon have to be the same (Polygon should "
        "be closed)");
  }

  for (const auto& point : points) {
    points_.emplace_back(point);

    auto boost_point = BoostPoint(point.getX(), point.getY());
    boost::geometry::append(polygon_.outer(), boost_point);
  }

  boost::geometry::correct(polygon_);
} /* -----  end of method Polygon::Polygon (constructor)  ----- */

Polygon Polygon::buildUnion(const Polygon& polygon) const {
  // Boost output polygon
  std::vector<BoostPolygon> output_polygons;

  // Union
  boost::geometry::union_(polygon_, polygon.polygon_, output_polygons);

  if (1 < output_polygons.size()) {
    throw std::runtime_error("Union results in more than one polygon.");
  }

  // Reference to the output polygon
  const auto& output_polygon = output_polygons[0];
  return getPolygonFromBoostPolygon(output_polygon);
}

std::vector<Point> Polygon::getIntersectionPoints(const Polygon& polygon) {
  std::vector<BoostPolygon> intersection_polygons;

  boost::geometry::intersection(polygon_, polygon.polygon_,
                                intersection_polygons);

  if (1 < intersection_polygons.size()) {
    throw std::runtime_error("Intersection results in more than one polygon.");
  }

  std::vector<Point> intersection_points;

  if (intersection_polygons.empty()) {
    return intersection_points;
  }

  // Reference to the points of the intersection polygon
  const auto& polygon_points = intersection_polygons[0].outer();

  for (std::vector<BoostPoint>::size_type i = 0; i < polygon_points.size();
       ++i) {
    intersection_points.emplace_back(
        Point(boost::geometry::get<0>(polygon_points[i]),
              boost::geometry::get<1>(polygon_points[i])));
  }

  return intersection_points;
}

int Polygon::getNumberOfIntersections(const Polygon& polygon) {
  auto intersection_points = getIntersectionPoints(polygon);

  if (intersection_points.empty()) {
    return 0;
  }

  return (intersection_points.size() - 1);
}

void Polygon::printIntersections(const Polygon& polygon) {
  auto intersection_points = getIntersectionPoints(polygon);
  if (!intersection_points.empty()) {
    std::cout << "Intersections: " << std::endl;

    for (const auto& point : intersection_points) {
      std::cout << "x: " << point.getX() << " y: " << point.getY() << std::endl;
    }
  }
}

Polygon Polygon::getPolygonFromBoostPolygon(const BoostPolygon& polygon) const {
  // Reference to the points of the input polygon
  const auto& polygon_points = polygon.outer();

  // Points to create a new Polygon
  std::vector<Point> points;

  for (std::vector<BoostPoint>::size_type i = 0; i < polygon_points.size();
       ++i) {
    points.emplace_back(Point(boost::geometry::get<0>(polygon_points[i]),
                              boost::geometry::get<1>(polygon_points[i])));
  }

  return Polygon(points);
}

std::vector<Point>& Polygon::getPoints() { return points_; }

void Polygon::print() {
  std::cout << "Polygon: " << std::endl;

  for (const auto& point : points_) {
    std::cout << "x: " << point.getX() << ", y: " << point.getY() << std::endl;
  }
}

void Polygon::plot(const std::string& filename) {
  // Declare a stream and an SVG mapper
  std::ofstream svg(filename);
  boost::geometry::svg_mapper<BoostPoint> mapper(svg, 800, 500);

  // Add geometries such that all these geometries fit on the map
  mapper.add(polygon_);

  // Draw the geometries on the SVG map, using a specific SVG style
  mapper.map(polygon_,
             "fill-opacity:0.3;fill:rgb(51,51,153);stroke:rgb(51,51,153);"
             "stroke-width:2");
}
