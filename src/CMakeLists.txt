ADD_EXECUTABLE(simple_example simple_example.cpp)
TARGET_LINK_LIBRARIES(simple_example octomap octomath)

ADD_EXECUTABLE(flo_test flo_test.cpp)
TARGET_LINK_LIBRARIES(flo_test octomap octomath)
