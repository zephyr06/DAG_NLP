#include "sources/TaskModel/GenerateRandomTaskset.h"
#include "sources/TaskModel/GenerateRandomTasksetWATERS.h"
#include "sources/Utils/argparse.hpp"
// #include "sources/Optimization/OptimizeOrder.h"
#include "sources/Baseline/RTSS21IC.h"
// #include "sources/RTA/RTA_DAG_Model.h"

using namespace GlobalVariablesDAGOpt;
void deleteDirectoryContents(const std::string &dir_path) {
  for (const auto &entry : std::filesystem::directory_iterator(dir_path))
    std::filesystem::remove_all(entry.path());
}
int main(int argc, char *argv[]) {
  using namespace std;
  argparse::ArgumentParser program("program name");
  program.add_argument("-v", "--verbose"); // parameter packing

  program.add_argument("--N").default_value(5).help("the number of tasks in DAG").scan<'i', int>();
  program.add_argument("--taskSetNumber")
      .default_value(10)
      .help("the number DAGs to create")
      .scan<'i', int>();
  program.add_argument("--taskSetNameStartIndex")
      .default_value(0)
      .help("the start index of DAG's name to create")
      .scan<'i', int>();
  program.add_argument("--numberOfProcessor")
      .default_value(4)
      .help("the numberOfProcessor of tasks in DAG")
      .scan<'i', int>();
  // NOTE!!!!!! THE actual utilization may be lower than the given values due to being upper-bounded by 1.0
  // for each task;
  program.add_argument("--totalUtilization")
      .default_value(0.4)
      .help("the total utilization of tasks in each DAG")
      .scan<'f', double>();
  program.add_argument("--aveUtilization")
      .default_value(0.0)
      .help("the average utilization of tasks in each core")
      .scan<'f', double>();
  program.add_argument("--useRandomUtilization")
      .default_value(1)
      .help("if 1, a random utilization in range [0.3 * numberOfProcessor, 0.9 "
            "* numberOfProcessor] will be "
            "used")
      .scan<'i', int>();
  program.add_argument("--minUtilizationPerCore")
      .default_value(0.3)
      .help("only used when --useRandomUtilization is 1")
      .scan<'f', double>();
  program.add_argument("--maxUtilizationPerCore")
      .default_value(0.9)
      .help("only used when --useRandomUtilization is 1")
      .scan<'f', double>();
  program.add_argument("--periodMin")
      .default_value(100)
      .help("the minimum period of tasks in DAG, used only when taskType is 0")
      .scan<'i', int>();
  program.add_argument("--periodMax")
      .default_value(500)
      .help("the maximum period of tasks in DAG, used only when taskType is 0")
      .scan<'i', int>();
  program.add_argument("--taskType")
      .default_value(2)
      .help("type of task period generation method, 0 means normal, 1 means random choice from predefined set, "
            "2 means automobile tasks with WATERS distribution")
      .scan<'i', int>();
  program.add_argument("--deadlineType")
      .default_value(0)
      .help("type of tasksets, 0 means implicit, 1 means random")
      .scan<'i', int>();
  program.add_argument("--taskSetType")
      .default_value(3)
      .help("type of tasksets, 0 means normal, "
            "1 means DAG with random chains, "
            "2 means DAG with chains conforms to WATERS15 distribution")
      .scan<'i', int>();
  program.add_argument("--coreRequireMax")
      .default_value(1)
      .help("maximum number of cores that each task could require, default 1")
      .scan<'i', int>();
  program.add_argument("--excludeUnschedulable")
      .default_value(1)
      .help("whether exclude unschedulable task set on List Scheduler, default "
            "1")
      .scan<'i', int>();
  program.add_argument("--excludeEmptyEdgeDag")
      .default_value(1)
      .help("whether exclude dags that don't have edges, default 1")
      .scan<'i', int>();
  program.add_argument("--randomSeed")
      .default_value(-1)
      .help("seed of random, negative means use current time as seed, "
            "otherwise means self-defined seed")
      .scan<'i', int>();
  program.add_argument("--outDir")
      .default_value(std::string("TaskData/dagTasks/"))
      .help("directory to save task sets, only within the root folder");
  program.add_argument("--clearOutputDir")
      .default_value(1)
      .help("whether clean the output directory")
      .scan<'i', int>();
  program.add_argument("--SF_ForkNum")
      .default_value(-1)
      .help("the number of forks that constitute sensor fusion objective "
            "functions, a negative number will generate random number of Fork "
            "range from 0.25N to N, default -1")
      .scan<'i', int>();
  program.add_argument("--fork_sensor_num_min")
      .default_value(2)
      .help("the minimum number of sensor tasks for each fork in SF "
            "experiments")
      .scan<'i', int>();
  program.add_argument("--fork_sensor_num_max")
      .default_value(9)
      .help("the maximum number of sensor tasks for each fork in SF "
            "experiments")
      .scan<'i', int>();
  program.add_argument("--numCauseEffectChain")
      .default_value(-1)
      .help("the number of random cause-effect chains, a negative number will"
            "use 1 to 2 times of tasks number as the number of chains")
      .scan<'i', int>();

  // program.add_argument("--parallelismFactor")
  //     .default_value(1000)
  //     .help("the parallelismFactor DAG")
  //     .scan<'i', int>();

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cout << err.what() << std::endl;
    std::cout << program;
    exit(0);
  }

  int N = program.get<int>("--N");
  int DAG_taskSetNumber = program.get<int>("--taskSetNumber");
  int DAG_taskSetNameStartIndex = program.get<int>("--taskSetNameStartIndex");
  int numberOfProcessor = program.get<int>("--numberOfProcessor");
  double totalUtilization;
  double aveUtilization = program.get<double>("--aveUtilization");
  int useRandomUtilization = program.get<int>("--useRandomUtilization");
  double minUtilizationPerCore = program.get<double>("--minUtilizationPerCore");
  double maxUtilizationPerCore = program.get<double>("--maxUtilizationPerCore");
  int periodMin = program.get<int>("--periodMin");
  int periodMax = program.get<int>("--periodMax");
  int taskType = program.get<int>("--taskType");
  int deadlineType = program.get<int>("--deadlineType");
  int taskSetType = program.get<int>("--taskSetType");
  int coreRequireMax = program.get<int>("--coreRequireMax");
  int excludeUnschedulable = program.get<int>("--excludeUnschedulable");
  int excludeEmptyEdgeDag = program.get<int>("--excludeEmptyEdgeDag");
  int randomSeed = program.get<int>("--randomSeed");
  std::string outDir = program.get<std::string>("--outDir");
  int clearOutputDir = program.get<int>("--clearOutputDir");
  int SF_ForkNum = program.get<int>("--SF_ForkNum");
  int fork_sensor_num_min = program.get<int>("--fork_sensor_num_min");
  int fork_sensor_num_max = program.get<int>("--fork_sensor_num_max");
  int numCauseEffectChain = program.get<int>("--numCauseEffectChain");

  if (randomSeed < 0) {
    srand(time(0));
  } else {
    srand(randomSeed);
  }
  if (aveUtilization != 0) {
    totalUtilization = aveUtilization * numberOfProcessor;
  } else {
    totalUtilization = program.get<double>("--totalUtilization");
  }
  {
    std::cout << "Task configuration: " << std::endl
              << "the number of tasks in DAG(--N): " << N << std::endl
              << "DAG_taskSetNumber(--taskSetNumber): " << DAG_taskSetNumber << std::endl
              << "DAG_taskSetNameStartIndex(--taskSetNameStartIndex): "
              << DAG_taskSetNameStartIndex << std::endl
              << "numberOfProcessor(--numberOfProcessor): " << numberOfProcessor << std::endl
              << "totalUtilization(--totalUtilization): " << totalUtilization << std::endl
              << "aveUtilization(--aveUtilization): " << aveUtilization << std::endl
              << "whether use random utilization(--useRandomUtilization): " << useRandomUtilization
              << std::endl
              << "minimum utilization per core, only work in random utilization "
                 "mode(--minUtilizationPerCore): "
              << minUtilizationPerCore << std::endl
              << "maximum utilization per core, only work in random utilization "
                 "mode(--maxUtilizationPerCore): "
              << maxUtilizationPerCore << std::endl
              << "periodMin, only work in normal(random) taskSetType(--periodMin), : " << periodMin
              << std::endl
              << "periodMax, only work in normal(random) taskSetType(--periodMax): " << periodMax << std::endl
              << "taskType(--taskType), 0 means normal, 1 means random choice from predefined set, "
              << "2 means automobile tasks with WATERS distribution: "
              << taskType << std::endl
              << "deadlineType(--deadlineType), 1 means random, 0 means implicit: " << deadlineType
              << std::endl
              << "taskSetType(--taskSetType), 0 means normal, "
              << "1 means DAG with random chains, "
              << "2 means DAG with chains conforms to WATERS15 distribution: "
              << taskSetType << std::endl
              << "coreRequireMax(--coreRequireMax): " << coreRequireMax << std::endl
              << "excludeUnschedulable(--excludeUnschedulable): " << excludeUnschedulable << std::endl
              << "excludeEmptyEdgeDag(--excludeEmptyEdgeDag): " << excludeEmptyEdgeDag << std::endl
              << "randomSeed, negative will use current time, otherwise use the "
                 "given seed(--randomSeed): "
              << randomSeed << std::endl
              << "outDir, directory to save task sets, only within the root folder "
                "(--outDir): "
              << outDir << std::endl
              << "clearOutputDir, whether clean the output directory "
                "(--clearOutputDir): "
              << clearOutputDir << std::endl
              << "SF_ForkNum, the number of forks, a negative number will generate random number of Fork "
                "range from 0.25N to N, default -1. (--SF_ForkNum): " << SF_ForkNum << std::endl
              << "the minimum number of sensor tasks for each fork in SF experiments "
                 "(--fork_sensor_num_min): "
              << fork_sensor_num_min << std::endl
              << "the minimum number of sensor tasks for each fork in SF experiments "
                 "(--fork_sensor_num_max): "
              << fork_sensor_num_max << std::endl
              << "numCauseEffectChain, the number of random cause-effect chains, a negative number "
                "will use 1 to 2 times of tasks number as the number of chains (--numCauseEffectChain): "
              << numCauseEffectChain << std::endl
              << std::endl;
  }
  std::string outDirectory = GlobalVariablesDAGOpt::PROJECT_PATH + outDir;
  // std::string outDirectory = GlobalVariablesDAGOpt::PROJECT_PATH +
  // "Energy_Opt_NLP/TaskData/task_number/";
  if (clearOutputDir > 0) {
    deleteDirectoryContents(outDirectory);
  }

  for (int i = DAG_taskSetNameStartIndex; i < DAG_taskSetNumber; i++) {
    if (useRandomUtilization) {
      totalUtilization =
          numberOfProcessor * (minUtilizationPerCore +
                               (double(rand()) / RAND_MAX) * (maxUtilizationPerCore - minUtilizationPerCore));
    }
    int targetNumCauseEffectChain;
    if (numCauseEffectChain >= 0) {
      targetNumCauseEffectChain = numCauseEffectChain;
    } else {
      targetNumCauseEffectChain = min(round(N * (1.0 + (double(rand()) / RAND_MAX) * 1.0 )), round(0.1 * N * N));
    }
    int targetSFForkNum;
    if (SF_ForkNum >= 0) {
      targetSFForkNum = SF_ForkNum;
    } else {
      targetSFForkNum = floor((0.25 + (double(rand()) / RAND_MAX) * 0.75 ) * N);
    }
    if (taskSetType == 0) // normal task set
    {
      CoutError("Not recognized type, needs implementation");
    } else if (taskSetType == 1 || taskSetType == 2) // DAG task set
    {
      DAG_Model dag_tasks;
      if (taskSetType == 1)
        dag_tasks = GenerateDAG_He21(N, totalUtilization, numberOfProcessor, periodMin, periodMax,
                                     coreRequireMax, targetSFForkNum, fork_sensor_num_min, fork_sensor_num_max,
                                     targetNumCauseEffectChain, taskType, deadlineType);
      else if (taskSetType == 2)
        dag_tasks = GenerateDAG_WATERS15(N, totalUtilization, numberOfProcessor, periodMin, periodMax,
                                         coreRequireMax, targetSFForkNum, fork_sensor_num_min, fork_sensor_num_max,
                                         targetNumCauseEffectChain, taskType, deadlineType);

      if (excludeEmptyEdgeDag == 1) {
        bool whether_empty_edges = true;
        for (auto pair : dag_tasks.mapPrev) {
          if (pair.second.size() > 0) {
            whether_empty_edges = false;
            break;
          }
        }
        if (whether_empty_edges) {
          i--;
          continue;
        }
      }

      if (targetSFForkNum > 0) {
        if (dag_tasks.sf_forks_.size() < targetSFForkNum) {
          i--;
          continue;
        }
      }
      if (targetNumCauseEffectChain > 0) {
        if (dag_tasks.chains_.size() < targetNumCauseEffectChain) {
          i--;
          continue;
        }
      }

      if (excludeUnschedulable == 1) {
        // rt_num_opt::RTA_DAG_Model rta(tasks);
        // std::cout << rta.CheckSchedulability() << std::endl;
        TaskSet &taskSet = dag_tasks.tasks;
        TaskSetInfoDerived tasksInfo(taskSet);
        std::vector<uint> processorJobVec;
        // std::optional<JobOrderMultiCore> emptyOrder;
        VectorDynamic initialSTV =
            ListSchedulingLFTPA(dag_tasks, tasksInfo, numberOfProcessor, processorJobVec);
        if (!ExamBasic_Feasibility(dag_tasks, tasksInfo, initialSTV, processorJobVec, numberOfProcessor)) {
          i--;
          continue;
        }
      }

      string fileName = "dag-set-N" + to_string(N) + "-" + string(3 - to_string(i).size(), '0') +
                        to_string(i) + "-syntheticJobs" + ".csv";
      std::ofstream myfile;
      myfile.open(outDirectory + fileName);
      WriteDAG(myfile, dag_tasks);
      myfile.close();
    }
  }

  return 0;
}
