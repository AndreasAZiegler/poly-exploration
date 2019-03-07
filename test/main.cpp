#include <glog/logging.h>
#include "gtest/gtest.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);  // init google logging
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
