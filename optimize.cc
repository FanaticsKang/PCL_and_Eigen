#include <iostream>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d &v) {
  Eigen::Matrix3d w;
  w << 0., -v(2), v(1), v(2), 0., -v(0), -v(1), v(0), 0.;
  return w;
}
Eigen::Vector3d ComputeValue(const Eigen::Vector3d &d, const Eigen::Vector3d &n,
                             const Eigen::Vector3d &pt) {
  return GetSkewMatrix(d) * pt + n;
}

int main() {
  Eigen::Vector3d p11, p12, p21, p22;
  Eigen::Vector3d n1, n2, d1, d2;
  p11 << 1, 0, 0;
  p12 << 1, 1, 0;

  d1 = p12 - p11;
  n1 = p11.cross(p12);
  std::cout << "n1: " << n1.transpose() << ", d1: " << d1.transpose()
            << ", p11: " << ComputeValue(d1, n1, p11).transpose()
            << ", p12: " << ComputeValue(d1, n1, p12).transpose() << std::endl;

  p21 << 0, 0, 1;
  p22 << 1, 0, 1;

  d2 = p22 - p21;
  n2 = p21.cross(p22);
  std::cout << "n2: " << n2.transpose() << ", d2: " << d2.transpose()
            << ", p21: " << ComputeValue(d2, n2, p21).transpose()
            << ", p22: " << ComputeValue(d2, n2, p22).transpose() << std::endl;

  Eigen::Matrix<double, 6, 3> A;
  A << GetSkewMatrix(d1), GetSkewMatrix(d2);

  Eigen::Matrix<double, 6, 1> B;
  B << -n1, -n2;

  std::cout << "result: " << A.colPivHouseholderQr().solve(B).transpose()
            << std::endl;
}