ADD_EXECUTABLE(simple_example simple_example.cpp)
TARGET_LINK_LIBRARIES(simple_example octomap octomath)

ADD_EXECUTABLE(flo_test flo_test.cpp)
TARGET_LINK_LIBRARIES(flo_test octomap octomath)

ADD_EXECUTABLE(test_algo test_algo.cpp MSP3D.cpp)
TARGET_LINK_LIBRARIES(test_algo octomap octomath kshortestpaths)

ADD_EXECUTABLE(test_on_map test_on_map.cpp MSP3D.cpp)
TARGET_LINK_LIBRARIES(test_on_map octomap octomath kshortestpaths)
