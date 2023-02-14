/**
   EN.601.463/663
   Assignment #1 

   Cartesian trajectory generation
   
 */
#include "assignment_3_4.hpp"

// Compute the forward kinematics (position and orientation)
// input: the joints angles
// output: the 4x4 homogeneous transformation
void ForwardKinematics( double q1, double q2, double q3, double E[4][4] ){
  
  // TODO
  // Fill the values of the forward kinematics (homogeneous matrix E)
	E[0][0] = (0.00848641014048002*sin(q2 + q3) + 0.999963989773096*cos(q2 + q3))*cos(q1);
	E[0][1] = -1.0*sin(q1);
	E[0][2] = (-0.999963989773096*sin(q2 + q3) + 0.00848641014048002*cos(q2 + q3))*cos(q1);
	E[0][3] = 0.109*sin(q1) - 0.213*sin(q2 + q3)*cos(q1) - 0.425*cos(q1)*cos(q2) - 0.486*cos(q1)*cos(q2 + q3);
	E[1][0] = (0.00848641014048002*sin(q2 + q3) + 0.999963989773096*cos(q2 + q3))*sin(q1);
	E[1][1] = 1.0*cos(q1);
	E[1][2] = (-0.999963989773096*sin(q2 + q3) + 0.00848641014048002*cos(q2 + q3))*sin(q1);
	E[1][3] = -0.213*sin(q1)*sin(q2 + q3) - 0.425*sin(q1)*cos(q2) - 0.486*sin(q1)*cos(q2 + q3) - 0.109*cos(q1);
	E[2][0] = 0.999963989773096*sin(q2 + q3) - 0.00848641014048002*cos(q2 + q3);
	E[2][1] = 0.0;
	E[2][2] = 0.00848641014048002*sin(q2 + q3) + 0.999963989773096*cos(q2 + q3);
	E[2][3] = -0.425*sin(q2) - 0.486*sin(q2 + q3) + 0.213*cos(q2 + q3) + 0.089;
	E[3][0] = 0.0;
	E[3][1] = 0.0;
	E[3][2] = 0.0;
	E[3][3] = 1.0;  
}

// Compute and return the Jacobian of the robot given the current joint 
// positions
// input: the joints angles
// output: the 6x3 Jacobian (position only)
void Jacobian( double q1, double q2, double q3, double J[6][3] ){
  
  // TODO
  // Fill the values of the Jacobian matrix J
	J[0][0] = 0.213*sin(q1)*sin(q2 + q3) + 0.425*sin(q1)*cos(q2) + 0.486*sin(q1)*cos(q2 + q3) + 0.109*cos(q1);
	J[0][1] = 0.425*sin(q2)*cos(q1) + 0.486*sin(q2 + q3)*cos(q1) - 0.213*cos(q1)*cos(q2 + q3);
	J[0][2] = 0.486*sin(q2 + q3)*cos(q1) - 0.213*cos(q1)*cos(q2 + q3);
	J[1][0] = 0.109*sin(q1) - 0.213*sin(q2 + q3)*cos(q1) - 0.425*cos(q1)*cos(q2) - 0.486*cos(q1)*cos(q2 + q3);
	J[1][1] = 0.425*sin(q1)*sin(q2) + 0.486*sin(q1)*sin(q2 + q3) - 0.213*sin(q1)*cos(q2 + q3);
	J[1][2] = 0.486*sin(q1)*sin(q2 + q3) - 0.213*sin(q1)*cos(q2 + q3);
	J[2][0] = 0.0;
	J[2][1] = -0.213*sin(q2 + q3) - 0.425*cos(q2) - 0.486*cos(q2 + q3);
	J[2][2] = -0.213*sin(q2 + q3) - 0.486*cos(q2 + q3);
	J[3][0] = 0.0;
	J[3][1] = 0.0;
	J[3][2] = 0.0;
	J[4][0] = 0.0;
	J[4][1] = 1.0;
	J[4][2] = 1.0;
	J[5][0] = 1.0;
	J[5][1] = 0.0;
	J[5][2] = 0.0;
}

