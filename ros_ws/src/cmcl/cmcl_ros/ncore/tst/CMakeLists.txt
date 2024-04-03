


add_executable(UnitTests UnitTests.cpp)
target_link_libraries(UnitTests ${OpenCV_LIBS} ${catkin_LIBRARIES} NCORE nlohmann_json::nlohmann_json GTest::GTest gtest_main ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} pybind11::embed)
target_compile_definitions(UnitTests PRIVATE PROJECT_TEST_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
add_test(AllTestsInTests UnitTests)


add_executable(BenchmarkClustering BenchmarkClustering.cpp TestUtils.cpp)
target_link_libraries(BenchmarkClustering ${OpenCV_LIBS} ${catkin_LIBRARIES} NCORE nlohmann_json::nlohmann_json ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} pybind11::embed)
target_compile_definitions(BenchmarkClustering PRIVATE PROJECT_TEST_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
#add_test(AllTestsInTests UnitTests)


add_executable(DumpClustering DumpClustering.cpp TestUtils.cpp)
target_link_libraries(DumpClustering ${OpenCV_LIBS} ${catkin_LIBRARIES} NCORE nlohmann_json::nlohmann_json ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} pybind11::embed)

