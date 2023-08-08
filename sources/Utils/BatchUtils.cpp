#include "sources/Utils/BatchUtils.h"

namespace OrderOptDAG_SPACE {

std::string GetResFileName(const std::string &pathDataset,
                           const std::string &file, BASELINEMETHODS method,
                           const std::string obj_trait) {
    std::string property;
    if (method < BaselineMethodNames.size())
        property =
            "_" + BaselineMethodNames[method] + "_" + obj_trait + "_Res.txt";
    else {
        CoutError("Unknown method index in GetResFileName!");
    }
    return pathDataset + file + property;
}

void WriteToResultFile(const std::string &pathDataset, const std::string &file,
                       const ScheduleResult &res, BASELINEMETHODS method,
                       const std::string obj_trait) {
    YAML::Node node;
    node["Schedulable"] = res.schedulable_;
    node["Obj_Overall"] = res.obj_;
    node["TimeTaken"] = res.timeTaken_;

    // The commented code can be considered to add
    //   if (batchTestMethod_ > 2) {
    //   resFile = resFile.substr(0, resFile.length() - 4);
    //   resFile += "_LoopCount.txt";
    //   outfileWrite.open(resFile, std::ofstream::out | std::ofstream::trunc);
    //   // std::ios_base::app outfileWrite << res.countOutermostWhileLoop_ <<
    //   std::endl; outfileWrite << res.countMakeProgress_ << std::endl;
    //   outfileWrite << res.countIterationStatus_ << std::endl;
    //   outfileWrite.close();
    // }

    std::string resFile = GetResFileName(pathDataset, file, method, obj_trait);
    std::ofstream fout(resFile);
    fout << node;
    fout.close();
}

ScheduleResult ReadFromResultFile(const std::string &pathDataset,
                                  const std::string &file,
                                  BASELINEMETHODS method,
                                  const std::string obj_trait) {
    std::string resFile = GetResFileName(pathDataset, file, method, obj_trait);
    YAML::Node node = YAML::LoadFile(resFile);

    ScheduleResult result;
    result.schedulable_ = node["Schedulable"].as<bool>();
    result.obj_ = node["Obj_Overall"].as<double>();
    result.timeTaken_ = node["TimeTaken"].as<double>();
    return result;
}

bool VerifyResFileExist(const std::string &pathDataset, const std::string &file,
                        BASELINEMETHODS method, const std::string obj_trait) {
    std::string resFile = GetResFileName(pathDataset, file, method, obj_trait);
    std::ifstream myfile;
    myfile.open(resFile);
    if (myfile) {
        return true;
    } else {
        return false;
    }
}

std::vector<std::string> ReadFilesInDirectory(const char *path) {
    std::vector<std::string> files;
    DIR *dr;
    struct dirent *en;
    dr = opendir(path);
    if (dr) {
        while ((en = readdir(dr)) != NULL) {
            files.push_back(en->d_name);  // print all directory name
        }
        closedir(dr);  // close all directory
    }
    std::sort(files.begin(), files.end());
    return files;
}
}  // namespace OrderOptDAG_SPACE