set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

include_directories(${gtest_SOURCE_DIR}/include)

find_package(yaml-cpp)

set(util_test_sources
    "tests_main_util.cpp"
    "tests_util.cpp")

add_executable(tests_util
    ${util_test_sources})
target_link_libraries(tests_util
    gtest_main
    yaml-cpp
    ${yaml-cpp_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES})

set(processing_test_sources
    "tests_main_processing.cpp"
    "tests_processing.cpp")

add_executable(tests_processing
    ${processing_test_sources})

target_link_libraries(tests_processing
    gtest_main
    yaml-cpp
    ${yaml-cpp_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES})

add_test(NAME PointCloudUtilTests
         COMMAND tests_util)
add_test(NAME PointCloudProcessingTests
         COMMAND tests_processing)
