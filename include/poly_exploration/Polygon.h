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

  void determinePolygonEdgeTypes();

  Polygon buildUnion(const Polygon& polygon) const;

  std::vector<Point> getIntersectionPoints(const Polygon& polygon) const;

  bool checkForIntersections(const Polygon& polygon) const;

  int getNumberOfIntersections(const Polygon& polygon) const;

  Polygon transformPolygon(const Pose& transformation) const;

  std::vector<PolygonPoint> getPoints() const;

  void setPointType(unsigned int point_id, PointType point_type);

  std::vector<Point> getXYPoints() const;

  std::vector<EdgeType> getEdgeTypes() const;

  void setPointTypesToPerformUnion();

  bool isPolygonFromSensorMeasurements() {
    return polygonFromSensorMeasurements;
  }

  void printIntersections(const Polygon& polygon) const;

  void print() const;

  friend std::ostream& operator<<(std::ostream& os, const Polygon& polygon);

  void plot(const std::string& filename) const;

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
