
#include "sources/Factors/AugmentedJacobian.h"

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
  BeginTimer("MergeAugJacobian");
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
  EndTimer("MergeAugJacobian");
  return jacobAll;
}
} // namespace OrderOptDAG_SPACE