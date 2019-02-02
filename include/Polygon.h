/* Polygon representation based on boost::Geometry.
 */

#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <vector>
#include <string>

#include "Point.h"

class Polygon {
 public:
  explicit Polygon(const std::vector<Point>& points);

  ~Polygon() = default;

  Polygon buildUnion(const Polygon& polygon) const;

  std::vector<Point> getIntersectionPoints(const Polygon& polygon);

  int getNumberOfIntersections(const Polygon& polygon);

  void printIntersections(const Polygon& polygon);

  std::vector<Point>& getPoints();

  void print();

  void plot(const std::string& filename);

 private:
  using BoostPoint = boost::geometry::model::d2::point_xy<double>;
  using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

  Polygon getPolygonFromBoostPolygon(const BoostPolygon& polygon) const;

  std::vector<Point> points_;
  BoostPolygon polygon_;
}; /* -----  end of class Polygon  ----- */
