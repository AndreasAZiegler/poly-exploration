find_package(GTest)

include(CodeCoverage)
set(COVERAGE_LCOV_EXCLUDES '/usr/include/*')

add_executable(poly_exploration-test
  main.cpp
  test-Polygon.cpp
  test-Point.cpp
  test-PolygonPoint.cpp
  test-PoseGraphPose.cpp
  test-PoseGraph.cpp
  test-PolygonConsolidation.cpp
  test-PolygonExplorerInterface.cpp
  ${PROJECT_SOURCE_DIR}/src/PolygonExplorer.cpp
  ${PROJECT_SOURCE_DIR}/src/Polygon.cpp
  ${PROJECT_SOURCE_DIR}/src/Point.cpp
  ${PROJECT_SOURCE_DIR}/src/PolygonPoint.cpp
  ${PROJECT_SOURCE_DIR}/src/PoseGraphPose.cpp
  ${PROJECT_SOURCE_DIR}/src/PoseGraph.cpp
  ${PROJECT_SOURCE_DIR}/src/PolygonConsolidation.cpp
)

target_link_libraries(poly_exploration-test
  PRIVATE
    gtest_main
    GTest::GTest
    GTest::Main
    ${GLOG_LIBRARIES}
    poly_exploration::poly_exploration
)


target_compile_features(poly_exploration-test PUBLIC cxx_std_17)
target_compile_options(poly_exploration-test PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
        -pipe -march=native
        $<$<CONFIG:Debug>:-g -ggdb3 -O0 -Wall -Wextra -fprofile-arcs -ftest-coverage>>
    $<$<CXX_COMPILER_ID:MSVC>:
        $<$<CONFIG:Debug>:/Od /Wall /Zi>>
)

gtest_add_tests(TARGET poly_exploration-test
                SOURCES
                main.cpp
                test-Point.cpp
                test-Polygon.cpp
                test-PolygonPoint.cpp
                test-PoseGraphPose.cpp
                test-PoseGraph.cpp
                test-PolygonConsolidation.cpp
                test-PolygonExplorerInterface.cpp
)

setup_target_for_coverage_lcov(NAME poly_exploration-test_coverage
                              EXECUTABLE poly_exploration-test
                              DEPENDENCIES poly_exploration-test)

