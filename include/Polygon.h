/* Polygon representation based on boost::Geometry.
 */

#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <string>
#include <vector>

#include "Geometry.h"
#include "Point.h"
#include "PolygonPoint.h"

enum class EdgeType { FRONTIER, OBSTACLE, FREE_SPACE };

std::ostream& operator<<(std::ostream& os, const EdgeType& edge_type);

class Polygon {
 public:
  explicit Polygon(const std::vector<PolygonPoint>& points);

  ~Polygon() = default;

  Polygon buildUnion(const Polygon& polygon) const;

  std::vector<Point> getIntersectionPoints(const Polygon& polygon);

  bool checkForIntersections(const Polygon& polygon);

  int getNumberOfIntersections(const Polygon& polygon);

  Polygon transformPolygon(const Pose& transformation);

  std::vector<PolygonPoint>& getPoints();

  std::vector<EdgeType>& getEdgeTypes();

  bool isPolygonFromSensorMeasurements() {
    return polygonFromSensorMeasurements;
  }

  void printIntersections(const Polygon& polygon);

  void print();

  void plot(const std::string& filename);

 private:
  explicit Polygon(const std::vector<Point>& points);

  using BoostPoint = boost::geometry::model::d2::point_xy<double>;
  using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

  Polygon getPolygonFromBoostPolygon(const BoostPolygon& polygon) const;

  bool polygonFromSensorMeasurements;

  std::vector<PolygonPoint> points_;
  BoostPolygon polygon_;

  std::vector<EdgeType> edgeTypes_;
}; /* -----  end of class Polygon  ----- */
