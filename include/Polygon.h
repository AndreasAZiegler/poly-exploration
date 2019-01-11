/* Polygon representation based on boost::Geometry.
 */

#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "Point.h"

class Polygon {
 public:
  Polygon(std::vector<Point> points);

  Polygon(const Polygon &other);

  ~Polygon();

  Polygon &operator=(const Polygon &other);

  Polygon buildUnion(const Polygon& polygon) const;

  std::vector<Point> &getPoints(void);

 private:
  using BoostPoint = boost::geometry::model::d2::point_xy<double>;
  using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

  Polygon getPolygonFromBoostPolygon(const BoostPolygon polygon) const;

  std::vector<Point> points_;
  BoostPolygon polygon_;

}; /* -----  end of class Polygon  ----- */
