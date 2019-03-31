find_package(Glog REQUIRED)

find_package(GTest REQUIRED)

find_package(kindr REQUIRED) 

find_package(Eigen3 REQUIRED  NO_MODULE)

add_library(poly_exploration
  #main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/PolygonExplorer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Polygon.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Point.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/PolygonPoint.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/PoseGraphPose.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/PoseGraph.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/PolygonConsolidation.cpp)

add_library(poly_exploration::poly_exploration ALIAS poly_exploration)

option(BUILD_SHARED_LIBS "Build shared library" ON)
include(GenerateExportHeader)
generate_export_header(poly_exploration
    EXPORT_MACRO_NAME POLY_EXPLORATION_API
    EXPORT_FILE_NAME ${CMAKE_BINARY_DIR}/include/poly_exploration/core/common.h
)

target_compile_features(poly_exploration PUBLIC cxx_std_17)
target_compile_options(poly_exploration PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
        -pipe -march=native
        $<$<CONFIG:Debug>:-g -ggdb3 -O0 -Wall -Wextra>>
    $<$<CXX_COMPILER_ID:MSVC>:
        $<$<CONFIG:Debug>:/Od /Wall /Zi>>
)

set_target_properties(poly_exploration PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_include_directories(poly_exploration
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/libs/eventpp-master/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/libs/kindr/include>
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(poly_exploration ${GLOG_LIBRARIES} Eigen3::Eigen)

include(GNUInstallDirs)

install(TARGETS poly_exploration
    EXPORT poly_exploration-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY
    ${CMAKE_SOURCE_DIR}/include/poly_exploration
    ${CMAKE_SOURCE_DIR}/include/eventpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT poly_exploration-targets
    FILE poly_exploration-targets.cmake
    NAMESPACE poly_exploration::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/poly_exploration
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/poly_exploration-config.cmake.in
    ${CMAKE_BINARY_DIR}/cmake/poly_exploration-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/poly_exploration
)

write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/cmake/poly_exploration-config-version.cmake
    VERSION ${POLY_EXPLORATION_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(
    FILES
        ${CMAKE_BINARY_DIR}/cmake/poly_exploration-config.cmake
        ${CMAKE_BINARY_DIR}/cmake/poly_exploration-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/poly_exploration
)

export(EXPORT poly_exploration-targets
    FILE ${CMAKE_BINARY_DIR}/cmake/poly_exploration-targets.cmake
    NAMESPACE poly_exploration::
)
