add_executable(poly-exploration
  main.cpp
  Polygon.cpp
  Point.cpp
  PolygonPoint.cpp
  PoseGraphPose.cpp
  PoseGraph.cpp
  PolygonConsolidation.cpp)

target_link_libraries(poly-exploration ${GLOG_LIBRARIES})
