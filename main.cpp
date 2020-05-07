#include "ViaPoint.h"

int main()
{
  Vector3d q_ini, q_via, q_fin, q_vel;

  q_ini << 0, 0, 0; q_via << 0.15, 0, 0.08; q_fin << 0, 0, 0; q_vel << 0.28, 0, 0.3;

  ViaPoint viapoint( 0.4, 1.0, q_ini, q_via, q_fin, q_vel );
  viapoint.set();
  viapoint.getTrajectory();

  return 0;
}
