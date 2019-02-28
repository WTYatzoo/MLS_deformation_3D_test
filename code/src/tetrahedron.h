#ifndef  _TETRAHEDRON_
#define _TETRAHEDRON_

#include </usr/include/eigen3/Eigen/Eigen>
#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/Cholesky>
#include </usr/include/eigen3/Eigen/LU>
#include </usr/include/eigen3/Eigen/Sparse>
#include </usr/include/eigen3/Eigen/SparseQR>
#include </usr/include/eigen3/Eigen/SparseLU>
#include </usr/include/eigen3/Eigen/IterativeLinearSolvers>
using namespace Eigen;

class tetrahedron
{
 public:
  int index_vertex[4];
  Matrix<double,3,3> P;
  int get_P;
  tetrahedron();
  ~tetrahedron();
};
#endif

