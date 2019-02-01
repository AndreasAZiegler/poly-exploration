add_executable(poly-exploration-test main.cpp ${PROJECT_SOURCE_DIR}/src/Polygon.cpp)

target_link_libraries(poly-exploration-test gtest_main GTest::GTest GTest::Main)

gtest_add_tests(TARGET poly-exploration-test
                SOURCES main.cpp
)
