gtsamAddTestsGlob(UnitTests "test*.cpp" "*RunSingle*" "${CONVENIENCE_LIB_NAME}")
gtsamAddTestsGlob(Single "test*RunSingle*.cpp" "" "${CONVENIENCE_LIB_NAME}")

# file(GLOB files "test*.cpp")
# foreach(file ${files})
#     ADD_EXECUTABLE(tBatch1 ${file})
#     TARGET_LINK_LIBRARIES(tBatch1 ${CONVENIENCE_LIB_NAME})
# endforeach()
# ADD_EXECUTABLE(testOpt testOptimize.cpp)
# TARGET_LINK_LIBRARIES(testOpt ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(testTask1 testTaskSetGenerate.cpp)
# TARGET_LINK_LIBRARIES(testTask1 ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(sa1 testOptimizeSA.cpp)
# TARGET_LINK_LIBRARIES(sa1 ${CONVENIENCE_LIB_NAME})


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



# **************************************************************

# set(RootVerucchi ${CMAKE_SOURCE_DIR}/sources/Baseline/Verucchi20)
# set(VARIABLE_SRCS	${RootVerucchi}/VariableTaskSet/VariableTaskSet.cpp
#                     ${RootVerucchi}/VariableTaskSet/VariableMultiEdge.cpp
# 					)

# set(MULTIRATE_SRCS	${RootVerucchi}/MultiRate/MultiNode.cpp
# ${RootVerucchi}/MultiRate/MultiEdge.cpp
# ${RootVerucchi}/MultiRate/MultiRateTaskset.cpp
# ${RootVerucchi}/MultiRate/DummyNodes.cpp
# 					)
					
# set(DAG_SRCS	${RootVerucchi}/DAG/DAG.cpp
# ${RootVerucchi}/DAG/Edge.cpp
# ${RootVerucchi}/DAG/MaxProduct.cpp
# 				)
					
# set(EVAL_SRCS	${RootVerucchi}/Evaluation/Evaluation.cpp
# ${RootVerucchi}/Evaluation/Scheduling.cpp
# 				)
# include_directories(${CMAKE_SOURCE_DIR}/sources/Baseline/Verucchi20)
# ADD_EXECUTABLE(tVerucchi tVerucchi.cpp ${VARIABLE_SRCS}
# ${MULTIRATE_SRCS}
# ${DAG_SRCS}
# ${EVAL_SRCS})
# ADD_EXECUTABLE(tVerucchi tVerucchi.cpp)
# TARGET_LINK_LIBRARIES(tVerucchi ${CONVENIENCE_LIB_NAME} )

# message(STATUS "0000000sources include: ${sources}")
# **************************************************************








# ADD_EXECUTABLE(testGraph1 testBGL_usage.cpp)
# TARGET_LINK_LIBRARIES(testGraph1 ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(saaa sa_functions.cpp)
# TARGET_LINK_LIBRARIES(saaa ${CONVENIENCE_LIB_NAME})

# ADD_EXECUTABLE(runSingle tRunSingle.cpp)
# TARGET_LINK_LIBRARIES(runSingle ${CONVENIENCE_LIB_NAME})

# ADD_EXECUTABLE(testDBF_M testDBF_Multi.cpp)
# TARGET_LINK_LIBRARIES(testDBF_M ${CONVENIENCE_LIB_NAME})

# include(GoogleTest)
# gtest_discover_tests(testdbfn)
# ADD_EXECUTABLE(exam example_boost.cpp)
# TARGET_LINK_LIBRARIES(exam ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(to2 testOptimize_coreNumber.cpp)
# TARGET_LINK_LIBRARIES(to2 ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(testFactor testTasks.cpp)
# TARGET_LINK_LIBRARIES(testTask ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(testRTA testResponseTimeAnalysis.cpp)
# TARGET_LINK_LIBRARIES(testRTA ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(testEnergy1 testEnergy.cpp)
# TARGET_LINK_LIBRARIES(testEnergy1 ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(testCompare1 testCompare.cpp)
# TARGET_LINK_LIBRARIES(testCompare1 ${CONVENIENCE_LIB_NAME})
# ADD_EXECUTABLE(testScratch1 testScratch.cpp)
# TARGET_LINK_LIBRARIES(testScratch1 ${CONVENIENCE_LIB_NAME})
