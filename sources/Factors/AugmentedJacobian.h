
#pragma once
#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Utils/MatirxConvenient.h"

namespace OrderOptDAG_SPACE {

// TODO: work with SM_Matrix
struct AugmentedJacobian {
  MatrixDynamic jacobian;
  VectorDynamic rhs;

  AugmentedJacobian() {}

  AugmentedJacobian(MatrixDynamic jacobian, VectorDynamic rhs) : jacobian(jacobian), rhs(rhs) {}

  AugmentedJacobian(int m, int n) : jacobian(GenerateMatrixDynamic(m, n)), rhs(GenerateVectorDynamic(m)) {}

  void print() {
    std::cout << "Jacobian: \n" << jacobian << std::endl;
    std::cout << "RHS: \n" << rhs << std::endl;
  }
};

// Warning! This function is slow because it creates a new AugmentedJacobian object
AugmentedJacobian StackAugJaco(const AugmentedJacobian &augJaco1, const AugmentedJacobian &augJaco2);

AugmentedJacobian MergeAugJacobian(const std::vector<AugmentedJacobian> &augJacos);
} // namespace OrderOptDAG_SPACE