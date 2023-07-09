#pragma once

#include "DeclareDAG.h"

#include "sources/Factors/RTDA_Analyze.h"
#include "sources/Factors/SensorFusion_Analyze.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/profilier.h"
namespace RTSS21IC_NLP {

namespace DAG_SPACE {
LLint CountSFError(OrderOptDAG_SPACE::DAG_Model &dagTasks, std::vector<LLint> &sizeOfVariables) {
  LLint errorDimensionSF = 0;
  for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++) {
    if ((itr->second).size() > 1)
      errorDimensionSF += sizeOfVariables[(itr->first)];
  }
  return errorDimensionSF + 2; // +2 for RTDA error
}
using namespace RegularTaskSystem;
struct IndexData {
  LLint index;
  double time;
};
std::pair<int, int> ExtractMaxDistance(std::vector<IndexData> &sourceFinishTime) {
  // return *max_element(sourceFinishTime.begin(), sourceFinishTime.end()) -
  //        *min_element(sourceFinishTime.begin(), sourceFinishTime.end());
  if (sourceFinishTime.size() == 0)
    return std::make_pair(0, 0);

  double maxEle = sourceFinishTime[0].time;
  int maxIndex = 0;
  double minEle = sourceFinishTime[0].time;
  int minIndex = 0;

  for (uint i = 0; i < sourceFinishTime.size(); i++) {
    if (maxEle < sourceFinishTime[i].time) {
      maxIndex = i;
      maxEle = sourceFinishTime[i].time;
    }
    if (minEle > sourceFinishTime[i].time) {
      minIndex = i;
      minIndex = sourceFinishTime[i].time;
    }
  }
  return std::make_pair(maxIndex, minIndex);
}
double ExtractMaxDistance(std::vector<double> &sourceFinishTime) {
  return *max_element(sourceFinishTime.begin(), sourceFinishTime.end()) -
         *min_element(sourceFinishTime.begin(), sourceFinishTime.end());
}
class SensorFusion_ConstraintFactor : public NoiseModelFactor1<VectorDynamic> {
public:
  OrderOptDAG_SPACE::DAG_Model dagTasks;
  TaskSet tasks;
  std::vector<LLint> sizeOfVariables;
  int N;
  double sensorFusionTol; // not used, actually
  LLint errorDimension;
  MAP_Index2Data mapIndex;
  std::vector<bool> maskForEliminate;
  LLint length;
  LLint lengthCompressed;
  std::unordered_map<LLint, LLint> mapIndex_True2Compress;
  OrderOptDAG_SPACE::TaskSetInfoDerived tasksInfo;

  SensorFusion_ConstraintFactor(Key key, OrderOptDAG_SPACE::DAG_Model &dagTasks,
                                std::vector<LLint> sizeOfVariables, LLint errorDimension,
                                double sensorFusionTol, MAP_Index2Data &mapIndex,
                                std::vector<bool> &maskForEliminate, SharedNoiseModel model)
      : NoiseModelFactor1<VectorDynamic>(model, key), dagTasks(dagTasks), tasks(dagTasks.tasks),
        sizeOfVariables(sizeOfVariables), N(tasks.size()), sensorFusionTol(sensorFusionTol),
        errorDimension(errorDimension), mapIndex(mapIndex), maskForEliminate(maskForEliminate) {
    length = 0;

    for (int i = 0; i < N; i++) {
      length += sizeOfVariables[i];
    }
    lengthCompressed = 0;
    for (LLint i = 0; i < length; i++) {
      if (maskForEliminate[i] == false)
        lengthCompressed++;
    }
    mapIndex_True2Compress = MapIndex_True2Compress(maskForEliminate);
    tasksInfo = OrderOptDAG_SPACE::TaskSetInfoDerived(dagTasks.tasks);
  }
  // TODO(Old): design some tests for SensorFusion
  Vector evaluateError(const VectorDynamic &startTimeVector,
                       boost::optional<Matrix &> H = boost::none) const override {
    BeginTimer("Sensor_all");

    if (H) {
      *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
    }
    EndTimer("Sensor_all");
    return f(startTimeVector);
  }

  boost::function<Matrix(const VectorDynamic &)> f = [this](const VectorDynamic &startTimeVectorOrig) {
    VectorDynamic startTimeVector = RecoverStartTimeVector(startTimeVectorOrig, maskForEliminate, mapIndex);

    VectorDynamic res = GenerateVectorDynamic(errorDimension);
    VectorDynamic sfError = OrderOptDAG_SPACE::ObtainSensorFusionError(dagTasks, tasksInfo, startTimeVector);

    if (dagTasks.chains_.size() == 0) {
      res(errorDimension - 2) = 0;
      res(errorDimension - 1) = 0;
      return res;
    }
    std::vector<OrderOptDAG_SPACE::RTDA> rtdaVec =
        OrderOptDAG_SPACE::GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[0], startTimeVector);
    OrderOptDAG_SPACE::RTDA rtda = GetMaxRTDA(rtdaVec);
    for (uint i = 0; i < errorDimension - 2; i++)
      res(i) = Barrier(RTSS21IC_NLP::sensorFusionTolerance - sfError(i));
    res(errorDimension - 2) = Barrier(RTSS21IC_NLP::freshTol - rtda.reactionTime);
    res(errorDimension - 1) = Barrier(RTSS21IC_NLP::freshTol - rtda.dataAge);
    // if (res.norm() != 0)
    //     int a = 1;
    return res;
  };
};

} // namespace DAG_SPACE
} // namespace RTSS21IC_NLP
