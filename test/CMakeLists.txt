add_executable(poly-exploration-test
  main.cpp
  test-Polygon.cpp
  test-Point.cpp
  test-PolygonPoint.cpp
  test-PoseGraphPose.cpp
  test-PoseGraph.cpp
  ${PROJECT_SOURCE_DIR}/src/Polygon.cpp
  ${PROJECT_SOURCE_DIR}/src/Point.cpp
  ${PROJECT_SOURCE_DIR}/src/PolygonPoint.cpp
  ${PROJECT_SOURCE_DIR}/src/PoseGraphPose.cpp
  ${PROJECT_SOURCE_DIR}/src/PoseGraph.cpp
  )

target_link_libraries(poly-exploration-test gtest_main GTest::GTest GTest::Main)

target_compile_options(poly-exploration-test PRIVATE -fprofile-arcs -ftest-coverage)

gtest_add_tests(TARGET poly-exploration-test
                SOURCES
                main.cpp
                test-Point.cpp
                test-Polygon.cpp
                test-PolygonPoint.cpp
                test-PoseGraphPose.cpp
                test-PoseGraph.cpp
)

SETUP_TARGET_FOR_COVERAGE_LCOV(NAME poly-exploration-test_coverage
                              EXECUTABLE poly-exploration-test
                              DEPENDENCIES poly-exploration-test)
