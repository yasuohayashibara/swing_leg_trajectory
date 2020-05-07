#include <iostream>
#include <vector>
#include <fstream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class ViaPoint
{
  private:
  double t_via, t_fin;
  Vector3d q_ini, q_via, q_fin, q_vel;
  Vector3d a3, a4, a5, pi1, pi2;
  Matrix<double, 3, 1> q;
  Matrix<double, 5, 3> param1, param2;
  vector<double> x_list, z_list;
  public:
  ViaPoint( double t_via, double t_fin, Vector3d q_ini, Vector3d q_via, Vector3d q_fin, Vector3d q_vel );
  void set();
  void getTrajectory();
};
