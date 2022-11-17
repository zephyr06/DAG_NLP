#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/Parameters.h"
#include "sources/Utils/BatchUtils.h"
#include "sources/TaskModel/DAG_Model.h"
TEST(parameters, a1)
{
    using namespace OrderOptDAG_SPACE;
    std::string dirStr = PROJECT_PATH + "TaskData/dagTasks/";
    const char *pathDataset = (dirStr).c_str();
    std::cout << "Dataset Directory: " << pathDataset << std::endl;
    std::vector<std::vector<double>> runTimeAll(TotalMethodUnderComparison);
    std::vector<std::vector<double>> objsAll(TotalMethodUnderComparison);
    std::vector<std::vector<int>> schedulableAll(TotalMethodUnderComparison); // values could only be 0 / 1

    std::vector<string> errorFiles;
    std::vector<string> worseFiles;
    std::vector<string> files = ReadFilesInDirectory(pathDataset);
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