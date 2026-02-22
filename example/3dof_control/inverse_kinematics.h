//
// Created by Milo on 2/16/26.
//

#ifndef INC_3DOF_CONTROL_INVERSE_KINEMATICS_H
#define INC_3DOF_CONTROL_INVERSE_KINEMATICS_H

struct JointAngles {
  double theta1;
  double theta2;
  double theta3;
};

class InverseKinematics {
private:
  JointAngles joint_angles = {0, 0, 0};

public:
  JointAngles solve_ik_3dof(double x, double y, double z);
};

#endif // INC_3DOF_CONTROL_INVERSE_KINEMATICS_H
