cmake_minimum_required(VERSION 2.8)

project(CargoBoxRecognition)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wno-deprecated-declarations")
enable_testing() # add this before add_subdirectory

find_package(yaml-cpp)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# For all header files with .hpp extensnion show
FILE(GLOB_RECURSE HPP_FILES "*.hpp")
add_custom_target(headers SOURCES ${HPP_FILES})

# For all header files with .hpp extensnion show
FILE(GLOB_RECURSE TCC_FILES "_impl/*.tcc")
add_custom_target(impls SOURCES ${TCC_FILES})

# Configuration filesl
FILE(GLOB_RECURSE CONFIG_FILES "config/*.yaml")
add_custom_target(configs SOURCES ${CONFIG_FILES})

add_executable(recognition
    "tool/recognition.cpp")

target_link_libraries(recognition
    yaml-cpp
    ${PCL_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
    ${Boost_LIBRARIES})

add_subdirectory(external)
add_subdirectory(tests)
