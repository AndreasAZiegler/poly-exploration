/*  PolygonExplorer unit test
 */

#include "PolygonExplorer.h"
#include "gtest/gtest.h"

// The fixture for testing class Polygon.
class PolygonExplorerTest : public ::testing::Test {
 protected:
  PolygonExplorerTest() = default;
};

TEST_F(PolygonExplorerTest, CreatePolygonExplorer) {
  PolygonExplorer polygon_explorer;
}
