/* Polygon representation based on boost::Geometry.
 */

#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <string>
#include <vector>

#include "Point.h"

class Polygon {
 public:
  explicit Polygon(const std::vector<Point>& points,
                   const std::vector<bool>& maximum_ranges);

  ~Polygon() = default;

  Polygon buildUnion(const Polygon& polygon) const;

  std::vector<Point> getIntersectionPoints(const Polygon& polygon);

  int getNumberOfIntersections(const Polygon& polygon);

  void printIntersections(const Polygon& polygon);

  std::vector<Point>& getPoints();

  bool isPolygonFromSensorMeasurements() {
    return polygonFromSensorMeasurements;
  }

  void print();

  void plot(const std::string& filename);

 private:
  explicit Polygon(const std::vector<Point>& points);

  using BoostPoint = boost::geometry::model::d2::point_xy<double>;
  using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

  Polygon getPolygonFromBoostPolygon(const BoostPolygon& polygon) const;

  bool polygonFromSensorMeasurements;

  std::vector<Point> points_;
  BoostPolygon polygon_;

  /// Indicates if sensor measurement is recorded at the maximum sensor range
  std::vector<bool> maximum_ranges_;
}; /* -----  end of class Polygon  ----- */
