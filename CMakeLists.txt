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

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# to use google test
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Boost - same requirement as gtsam
find_package(Boost 1.50 REQUIRED)
find_package(CppUnitLite REQUIRED)
find_package( OpenCV REQUIRED )
include_directories(${Boost_INCLUDE_DIR})
include_directories(includeMoe)
include_directories(/usr/local/lib)
# file(GLOB sources "*.cpp" "*.h")
file(GLOB sources "sources/*.cpp" "sources/*.h")
set(CONVENIENCE_LIB_NAME ${PROJECT_NAME})
add_library(${CONVENIENCE_LIB_NAME} SHARED ${sources})

# linking with dependencies
target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)
target_link_libraries(${CONVENIENCE_LIB_NAME} ${Boost_LIBRARIES})
target_link_libraries(${CONVENIENCE_LIB_NAME} CppUnitLite)
target_link_libraries(${CONVENIENCE_LIB_NAME} ${OpenCV_LIBS} )
target_link_libraries(${CONVENIENCE_LIB_NAME} gtest_main )


add_subdirectory(tests)
add_subdirectory(sources)
