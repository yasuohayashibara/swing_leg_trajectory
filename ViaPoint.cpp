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
  Matrix<double, 5, 3> param1;
  param1 <<
    (q_via-q_ini).transpose(),
	(q_fin-q_ini).transpose(),
	q_vel.transpose(),
	0, 0, 0,
	0, 0, 0;

  Matrix<double, 5, 5> A;
  A <<
      pow(t_via,3),    pow(t_via,4),    pow(t_via,5),                       0,                        0,
	  pow(t_fin,3),    pow(t_fin,4),    pow(t_fin,5), pow((t_fin-t_via),4)/24, pow((t_fin-t_via),5)/120,
	3*pow(t_via,2),  4*pow(t_via,3),  5*pow(t_via,4),                       0,                        0,
	3*pow(t_fin,2),  4*pow(t_fin,3),  5*pow(t_fin,4), pow((t_fin-t_via),3)/ 6, pow((t_fin-t_via),4)/ 24,
	6*pow(t_fin,1), 12*pow(t_fin,2), 20*pow(t_fin,3), pow((t_fin-t_via),2)/ 2, pow((t_fin-t_via),3)/  6;

  param2 = A.inverse()*param1;
  a3  = param2.row(0);
  a4  = param2.row(1);
  a5  = param2.row(2);
  pi1 = param2.row(3);
  pi2 = param2.row(4);
}

void ViaPoint::getTrajectory()
{
  ofstream result( "result.csv" );

  for( double t_now = 0; t_now <= t_fin; t_now += 0.01 ) {
	if( t_now <= t_via ) {
	  q << a5*pow(t_now,5) + a4*pow(t_now,4) + a3*pow(t_now,3) + q_ini;
	} else {
	  q << a5*pow(t_now,5) + a4*pow(t_now,4) + a3*pow(t_now,3) + q_ini + (pi1/24)*pow(t_now-t_via,4) + (pi2/120)*pow(t_now-t_via,5);
	}
	result << t_now << ", " << q(0) << ", " << q(1) << ", " << q(2) << ", " << endl;
  }
  result.close();
}
