/* Polygon explorer interface.
 *
 * Abstract class defining the interface with the visualization update callback
 * method
 */

#pragma once

#include "poly_exploration/PoseGraph.h"

class PolygonExplorerInterface {
 public:
  virtual void updateVisualizationCallback(const PoseGraph) = 0;
};
