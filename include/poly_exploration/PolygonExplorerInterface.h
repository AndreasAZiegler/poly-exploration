/* Polygon explorer interface.
 *
 * Abstract class defining the interface with the visualization update callback
 * method
 */

#pragma once

#include "poly_exploration/PoseGraph.h"

struct TimeStamp {
  int sec;
  unsigned int nanosec;
};

class PolygonExplorerInterface {
 public:
  virtual void updateVisualizationCallback(PoseGraph pose_graph,
                                           TimeStamp time_stamp) = 0;
};
