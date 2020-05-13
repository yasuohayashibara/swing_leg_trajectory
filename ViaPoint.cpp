#include "ViaPoint.h"

ViaPoint::ViaPoint( double t_via, double t_fin, Vector3d q_ini, Vector3d q_via, Vector3d q_fin, Vector3d q_vel )
{
  this->t_via = t_via;
  this->t_fin = t_fin;
  this->q_ini = q_ini;
  this->q_via = q_via;
  this->q_fin = q_fin;
  this->q_vel = q_vel;
}

void ViaPoint::set()
{
  Matrix<double, 5, 5> A;

  param1 << q_via(0)-q_ini(0), q_via(1)-q_ini(1), q_via(2)-q_ini(2),
	  q_fin(0)-q_ini(0), q_fin(1)-q_ini(1), q_fin(2)-q_ini(2),
	  q_vel(0), q_vel(1), q_vel(2),
	  0, 0, 0,
	  0, 0, 0;

  A << pow(t_via,3), pow(t_via,4), pow(t_via,5), 0, 0,
	pow(t_fin,3), pow(t_fin,3), pow(t_fin,3), pow((t_fin-t_via),4)/24, pow((t_fin-t_via),5)/120,
	3*pow(t_via,2), 4*pow(t_via,3), 5*pow(t_via,4), 0, 0,
	3*pow(t_fin,2), 4*pow(t_fin,3), 5*pow(t_fin,4), pow((t_fin-t_via),3)/6, pow((t_fin-t_via),4)/24,
	6*pow(t_fin,2), 12*pow(t_fin,3), 20*pow(t_fin,4), pow((t_fin-t_via),2)/2, pow((t_fin-t_via),3)/6;

  param2 = A.inverse()*param1;
  cout << param2 << endl;
  a3 = param2.row(0); a4 = param2.row(1); a5 = param2.row(2);
  pi1 = param2.row(3); pi2 = param2.row(4);
}

void ViaPoint::getTrajectory()
{
  ofstream data1( "data_x.txt" );
  ofstream data2( "data_z.txt" );

  for( double t_now=0; t_now<=t_fin; t_now+=0.01 ) {
	if( t_now>=0 && t_now<=t_via ) {
	  q << a5(0)*pow(t_now,5) + a4(0)*pow(t_now,4) + a3(0)*pow(t_now,3) + q_ini(0),
		a5(1)*pow(t_now,5) + a4(1)*pow(t_now,4) + a3(1)*pow(t_now,3) + q_ini(1),
		a5(2)*pow(t_now,5) + a4(2)*pow(t_now,4) + a3(2)*pow(t_now,3) + q_ini(2);

	  data1 << q(0) << endl;
	  data2 << q(2) << endl;
	}
	else if( t_now>t_via && t_now<= t_fin ) {
	  q << a5(0)*pow(t_now,5) + a4(0)*pow(t_now,4) + a3(0)*pow(t_now,3) + q_ini(0) + (pi1(0)/24)*pow(t_now-t_via,4) + (pi2(0)/120)*pow(t_now-t_via,5),
		a5(1)*pow(t_now,5) + a4(1)*pow(t_now,4) + a3(1)*pow(t_now,3) + q_ini(1) + (pi1(1)/24)*pow(t_now-t_via,4) + (pi2(1)/120)*pow(t_now-t_via,5),
		a5(2)*pow(t_now,5) + a4(2)*pow(t_now,4) + a3(2)*pow(t_now,3) + q_ini(2) + (pi1(2)/24)*pow(t_now-t_via,4) + (pi2(2)/120)*pow(t_now-t_via,5);

	  data1 << q(0) << endl;
	  data2 << q(2) << endl;
	}
  }
  data1.close();
  data2.close();
}
