#include "sources/Optimization/InitialEstimate.h"

using namespace RegularTaskSystem;
namespace OrderOptDAG_SPACE
{

    gtsam::Values GenerateInitialFG(const VectorDynamic &startTimeVector, const TaskSetInfoDerived &tasksInfo)
    {
        gtsam::Values initialEstimateFG;
        gtsam::Symbol key('a', 0); // just declare the variable

        for (int i = 0; i < tasksInfo.N; i++)
        {
            for (int j = 0; j < int(tasksInfo.sizeOfVariables.at(i)); j++)
            {
                // LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);

                gtsam::Symbol key = GenerateKey(i, j);
                VectorDynamic v = GenerateVectorDynamic(1);
                v << ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j);

                initialEstimateFG.insert(key, v);
            }
        }
        return initialEstimateFG;
    }

    /**
     * @brief Generate initial estimate based on provided options'
     * param: initializeMethod: global parameter passed implicitly
     *
     * @param dagTasks
     * @param sizeOfVariables
     * @param variableDimension
     * @return VectorDynamic
     */
    VectorDynamic GenerateInitial(DAG_Model &dagTasks,
                                  std::vector<LLint> &sizeOfVariables,
                                  int variableDimension, VectorDynamic initialUser)
    {
        TaskSetInfoDerived tasksInfo(dagTasks.tasks);
        VectorDynamic initialEstimate;
        if (initialUser.norm() != 0)
        {
            LLint size = GenerateInitial(dagTasks, sizeOfVariables, variableDimension).rows();
            initialEstimate = initialUser;
            if (initialUser.rows() != size)
            {
                CoutError("User input initial vector has wrong length!");
            }
            return initialUser;
        }
        initialEstimate = ListSchedulingLFTPA(dagTasks,
                                              tasksInfo, GlobalVariablesDAGOpt::coreNumberAva);
        return initialEstimate;
    }

}