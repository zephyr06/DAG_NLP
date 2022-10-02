cmake_minimum_required(VERSION 3.0)
project(EnergyOptNLP CXX C)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

include_directories(${CMAKE_SOURCE_DIR})
include_directories("/usr/include/opencv4/opencv2")
include_directories("/usr/include/opencv4")

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")
set(CMAKE_WARN_DEPRECATED OFF)
set(CMAKE_ERROR_DEPRECATED OFF)
add_compile_options(-Wdeprecated-declarations)
unset(CMAKE_WARN_DEPRECATED)

# Boost - same requirement as gtsam
find_package(Boost 1.50 REQUIRED)
find_package(CppUnitLite REQUIRED)
find_package( OpenCV REQUIRED )
find_package(yaml-cpp)
include_directories(${Boost_INCLUDE_DIR})
include_directories(includeMoe)
include_directories(/usr/local/lib)
# file(GLOB sources "sources/*.cpp" "sources/*.h")
file(GLOB_RECURSE sources "sources/*.cpp" "sources/*.h")
set(CONVENIENCE_LIB_NAME ${PROJECT_NAME})
add_library(${CONVENIENCE_LIB_NAME} SHARED ${sources})

# linking with dependencies
target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)
target_link_libraries(${CONVENIENCE_LIB_NAME} ${Boost_LIBRARIES})
target_link_libraries(${CONVENIENCE_LIB_NAME} CppUnitLite)
target_link_libraries(${CONVENIENCE_LIB_NAME} ${OpenCV_LIBS} )
target_link_libraries(${CONVENIENCE_LIB_NAME} yaml-cpp)
target_link_libraries(${CONVENIENCE_LIB_NAME} "stdc++fs")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(tests)
add_subdirectory(sources)
add_subdirectory(${CMAKE_SOURCE_DIR}/sources/Baseline/Verucchi20)