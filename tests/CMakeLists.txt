
gtsamAddTestsGlob(UnitTests "test*.cpp" "" "${CONVENIENCE_LIB_NAME}")



ADD_EXECUTABLE(GenerateTaskSet GenerateTaskSet.cpp)
TARGET_LINK_LIBRARIES(GenerateTaskSet ${CONVENIENCE_LIB_NAME})
