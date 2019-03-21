add_executable(poly-exploration
  main.cpp
  PolygonExplorer.cpp
  Polygon.cpp
  Point.cpp
  PolygonPoint.cpp
  PoseGraphPose.cpp
  PoseGraph.cpp
  PolygonConsolidation.cpp)

target_link_libraries(poly-exploration ${GLOG_LIBRARIES})
