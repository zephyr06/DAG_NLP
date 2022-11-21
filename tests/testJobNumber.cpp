#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/Parameters.h"
#include "sources/Utils/BatchUtils.h"
#include "sources/TaskModel/DAG_Model.h"
using namespace std;
using namespace OrderOptDAG_SPACE;
using namespace GlobalVariablesDAGOpt;
TEST(parameters, a1)
{

    std::string dirStr = PROJECT_PATH + "TaskData/dagTasks/";
    const char *pathDataset = (dirStr).c_str();
    std::cout << "Dataset Directory: " << pathDataset << std::endl;
    std::vector<std::vector<double>> runTimeAll(TotalMethodUnderComparison);
    std::vector<std::vector<double>> objsAll(TotalMethodUnderComparison);
    std::vector<std::vector<int>> schedulableAll(TotalMethodUnderComparison); // values could only be 0 / 1

    std::vector<std::string> errorFiles;
    std::vector<std::string> worseFiles;
    std::vector<std::string> files = ReadFilesInDirectory(pathDataset);
    std::vector<int> jobNumber;
    for (const auto &file : files)
    {
        string delimiter = "-";
        if (file.substr(0, file.find(delimiter)) == "dag" && file.find("Res") == std::string::npos)
        {
            string path = PROJECT_PATH + "TaskData/dagTasks/" + file;
            OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(path, priorityMode);
            RegularTaskSystem::TaskSetInfoDerived tasksInfo(dagTasks.tasks);

            jobNumber.push_back(tasksInfo.length);
        }
    }
    std::cout << "Job number" << Average(jobNumber) << std::endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}