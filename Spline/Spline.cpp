/* 5次スプライン曲線 */
#include <iostream>
#include <vector>
#include <fstream>

#include <Eigen/Dense>
//#include "matplotlibcpp.h"

using namespace std;
using namespace Eigen;
//namespace plt = matplotlibcpp;

int main()
{
  Matrix<double, 1, 6> x_point;  x_point << 0.0, -3.5, -1.2, 2.0, 2.5, 0.0;
  Matrix<double, 1, 6> z_point;  z_point << 0.0, 2.0, 1.6, 0.8, 3.0, 0.0;

  double temp_u[5];
  double u[6];
  Matrix<double, 6, 6> param1;
  Matrix<double, 6, 1> param_x;
  Matrix<double, 6, 1> param_z;
  vector<double> x_list, z_list;

  for( int i=0; i<5; i++ ) {
	temp_u[i] = sqrt( pow(x_point[i+1]-x_point[i],2) + pow(z_point[i+1]-z_point[i],2) );
  }

  u[0] = 0;
  for( int i=1; i<7; i++ ) {
	u[i] = u[i-1] + temp_u[i-1];
  }

  param1 << pow(u[0],5), pow(u[0],4), pow(u[0],3), pow(u[0],2), u[0], 1,
  pow(u[1],5), pow(u[1],4), pow(u[1],3), pow(u[1],2), u[1], 1,
  pow(u[2],5), pow(u[2],4), pow(u[2],3), pow(u[2],2), u[2], 1,
  pow(u[3],5), pow(u[3],4), pow(u[3],3), pow(u[3],2), u[3], 1,
  pow(u[4],5), pow(u[4],4), pow(u[4],3), pow(u[4],2), u[4], 1,
  pow(u[5],5), pow(u[5],4), pow(u[5],3), pow(u[5],2), u[5], 1;

  param_x = param1.inverse()*x_point.transpose();
  param_z = param1.inverse()*z_point.transpose();

  double sum;
  for( int i=0; i<5; i++ ) {
	sum += temp_u[i];
  }

  ofstream data1( "data_x.txt" );
  ofstream data2( "data_z.txt" );
  for( double t=0; t<=sum+(sum/100); t+=(sum/100) ) {
	x_list.push_back( param_x(0)*pow(t,5) + param_x(1)*pow(t,4) + param_x(2)*pow(t,3) + param_x(3)*pow(t,2) + param_x(4)*t + param_x(5) );
	z_list.push_back( param_z(0)*pow(t,5) + param_z(1)*pow(t,4) + param_z(2)*pow(t,3) + param_z(3)*pow(t,2) + param_z(4)*t + param_z(5) );
	data1 << (param_x(0)*pow(t,5) + param_x(1)*pow(t,4) + param_x(2)*pow(t,3) + param_x(3)*pow(t,2) + param_x(4)*t + param_x(5))  << endl;
	data2 << (param_z(0)*pow(t,5) + param_z(1)*pow(t,4) + param_z(2)*pow(t,3) + param_z(3)*pow(t,2) + param_z(4)*t + param_z(5))  << endl;
  }

/*
  plt::plot( x_list, z_list );
  plt::show();
*/
  
  return 0;
}
