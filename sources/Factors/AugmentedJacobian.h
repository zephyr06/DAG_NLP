
#pragma once
#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Utils/MatirxConvenient.h"

namespace OrderOptDAG_SPACE {

typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double> EigenTriplet;
typedef std::vector<EigenTriplet> EigenTripletVector;

struct AugmentedJacobianTriplet {
  EigenTripletVector jacobian;
  VectorDynamic rhs;

  AugmentedJacobianTriplet() {}

  SpMat GetSpMat() const {
    int m = 0, n = 0;
    for (uint i = 0; i < jacobian.size(); i++) {
      m = std::max(m, jacobian[i].row());
      n = std::max(n, jacobian[i].col());
    }
    SpMat mat(m + 1, n + 1);
    mat.setFromTriplets(jacobian.begin(), jacobian.end());
    return mat;
  }

  MatrixDynamic GetDenseMat() const { return MatrixDynamic(GetSpMat()); }

  void print() {

    std::cout << "Jacobian: \n" << GetDenseMat() << std::endl;
    std::cout << "RHS: \n" << rhs << std::endl;
  }
};

// TODO: work with SM_Matrix
struct AugmentedJacobian {
  MatrixDynamic jacobian;
  VectorDynamic rhs;

  AugmentedJacobian() {}

  AugmentedJacobian(const AugmentedJacobianTriplet &augTrip)
      : jacobian(augTrip.GetDenseMat()), rhs(augTrip.rhs) {}

  AugmentedJacobian(MatrixDynamic jacobian, VectorDynamic rhs) : jacobian(jacobian), rhs(rhs) {}

  AugmentedJacobian(int m, int n) : jacobian(GenerateMatrixDynamic(m, n)), rhs(GenerateVectorDynamic(m)) {}

  void print() {
    std::cout << "Jacobian: \n" << jacobian << std::endl;
    std::cout << "RHS: \n" << rhs << std::endl;
  }
};

struct AugmentedSparseJacobian {
  SpMat jacobian;
  VectorDynamic rhs;

  AugmentedSparseJacobian() {}

  AugmentedSparseJacobian(SpMat jacobian, VectorDynamic rhs) : jacobian(jacobian), rhs(rhs) {}

  void print() {
    std::cout << "Jacobian: \n" << MatrixDynamic(jacobian) << std::endl;
    std::cout << "RHS: \n" << rhs << std::endl;
  }
};

// Warning! This function is slow because it creates a new AugmentedJacobian object
AugmentedJacobian StackAugJaco(const AugmentedJacobian &augJaco1, const AugmentedJacobian &augJaco2);

AugmentedJacobian MergeAugJacobian(const std::vector<AugmentedJacobian> &augJacos);

} // namespace OrderOptDAG_SPACE