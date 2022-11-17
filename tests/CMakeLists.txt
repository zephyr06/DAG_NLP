gtsamAddTestsGlob(Single "*RunSingle*.cpp" "" "${CONVENIENCE_LIB_NAME}")
gtsamAddTestsGlob(UnitTests "test*.cpp" "*RunSingle*" "${CONVENIENCE_LIB_NAME}")



ADD_EXECUTABLE(GenerateTaskSet GenerateTaskSet.cpp)
TARGET_LINK_LIBRARIES(GenerateTaskSet ${CONVENIENCE_LIB_NAME})

# TODO: figure out why can't this command work!
# ADD_EXECUTABLE(tBatch1 testBatch.cpp)
# TARGET_LINK_LIBRARIES(tBatch1 ${CONVENIENCE_LIB_NAME})

# ADD_EXECUTABLE(tBatchVerucchi testBatchVerucchi.cpp)
# TARGET_LINK_LIBRARIES(tBatchVerucchi ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(tBatchWangNLP testBatchWangNLP.cpp)
# TARGET_LINK_LIBRARIES(tBatchWangNLP ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(tBatchOrderOpt testBatchOrderOpt.cpp)
# TARGET_LINK_LIBRARIES(tBatchOrderOpt ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(tBatchTOM testBatchTOM.cpp)
# TARGET_LINK_LIBRARIES(tBatchTOM ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(tBatchOrderOpt1LP testBatchOrderOpt1LP.cpp)
# TARGET_LINK_LIBRARIES(tBatchOrderOpt1LP ${CONVENIENCE_LIB_NAME})