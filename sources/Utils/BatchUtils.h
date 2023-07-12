#pragma once

#include <string.h>

#include "sources/Utils/Parameters.h"
// #include "sources/Optimization/OptimizeOrder.h"
#include "sources/BaselineMethods.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "yaml-cpp/yaml.h"

namespace OrderOptDAG_SPACE {

std::string GetResFileName(const std::string &pathDataset,
                           const std::string &file, BASELINEMETHODS method,
                           const std::string obj_trait);

void WriteToResultFile(const std::string &pathDataset, const std::string &file,
                       const ScheduleResult &res, BASELINEMETHODS method,
                       const std::string obj_trait);

ScheduleResult ReadFromResultFile(const std::string &pathDataset,
                                  const std::string &file,
                                  BASELINEMETHODS method,
                                  const std::string obj_trait);

bool VerifyResFileExist(const std::string &pathDataset, const std::string &file,
                        BASELINEMETHODS method, const std::string obj_trait);

std::vector<std::string> ReadFilesInDirectory(const char *path);

void WriteScheduleToFile(const std::string &pathDataset,
                         const std::string &file, DAG_Model &dagTasks,
                         ScheduleResult &res, int batchTestMethod_);

}  // namespace OrderOptDAG_SPACE