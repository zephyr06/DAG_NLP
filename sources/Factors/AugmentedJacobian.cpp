
#include "sources/Factors/AugmentedJacobian.h"
#include "sources/Utils/profilier.h"

namespace OrderOptDAG_SPACE {
AugmentedJacobian StackAugJaco(const AugmentedJacobian &augJaco1, const AugmentedJacobian &augJaco2) {
  MatrixDynamic jacobianAll(augJaco1.jacobian.rows() + augJaco2.jacobian.rows(), augJaco1.jacobian.cols());
  jacobianAll << augJaco1.jacobian, augJaco2.jacobian;

  VectorDynamic rhsAll(augJaco1.rhs.rows() + augJaco2.rhs.rows(), 1);
  rhsAll << augJaco1.rhs, augJaco2.rhs;
  return AugmentedJacobian{jacobianAll, rhsAll};
}

// TODO: this function could possibly improve efficiency?
// TODO: switch Jacobian to sparseMatrix
AugmentedJacobian MergeAugJacobian(const std::vector<AugmentedJacobian> &augJacos) {
#ifdef PROFILE_CODE
  BeginTimer(__FUNCTION__);
#endif
  AugmentedJacobian jacobAll;
  if (augJacos.size() == 0)
    return jacobAll;

  int n = augJacos[0].jacobian.cols();
  int totalRow = 0;
  for (uint i = 0; i < augJacos.size(); i++)
    totalRow += augJacos[i].jacobian.rows();

  jacobAll.jacobian.conservativeResize(totalRow, n);
  jacobAll.rhs.conservativeResize(totalRow, 1);
  int rowCount = 0;
  for (uint i = 0; i < augJacos.size(); i++) {
    jacobAll.jacobian.block(rowCount, 0, augJacos[i].jacobian.rows(), n) = augJacos[i].jacobian;
    jacobAll.rhs.block(rowCount, 0, augJacos[i].rhs.rows(), 1) = augJacos[i].rhs;
    rowCount += augJacos[i].jacobian.rows();
  }
#ifdef PROFILE_CODE
  EndTimer(__FUNCTION__);
#endif
  return jacobAll;
}

AugmentedSparseJacobian MergeAugJacobian(const std::vector<AugmentedJacobianTriplet> &augJacos) {
  int totalRow = 0;
  int totalCol = 0;
  for (uint i = 0; i < augJacos.size(); i++) {
    totalRow += augJacos[i].rows;
    totalCol = std::max(totalCol, augJacos[i].cols);
  }

  EigenTripletVector tripletVecAll;
  tripletVecAll.reserve(totalRow);
  VectorDynamic rhsAll(totalRow);
  int rowIndex = 0;

  for (uint i = 0; i < augJacos.size(); i++) {
    const AugmentedJacobianTriplet &augJacobCurr = augJacos[i];
    for (uint j = 0; j < augJacobCurr.jacobian.size(); j++) {
      EigenTripletMy tripCurr = augJacobCurr.jacobian[j];
      tripCurr.UpdateRow(tripCurr.row() + rowIndex);
      tripletVecAll.push_back(tripCurr);
    }
    rhsAll.block(rowIndex, 0, augJacobCurr.rhs.rows(), 1) = augJacobCurr.rhs;
    rowIndex += augJacos[i].rows;
  }

  AugmentedSparseJacobian augSparJacob;
  augSparJacob.jacobian = SpMat(totalRow, totalCol);
  augSparJacob.jacobian.setFromTriplets(tripletVecAll.begin(), tripletVecAll.end());
  augSparJacob.rhs = rhsAll;
  return augSparJacob;
}
} // namespace OrderOptDAG_SPACE