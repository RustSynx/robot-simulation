//
// Created by Milo on 2/22/26.
//

#include "inverse_kinematics.h"

#include <cmath>

/**
 * Calculates Inverse Kinematics for the UR5e robot.
 * Link lengths (meters): L1=0.425, L2=0.392
 * @param x target x coordinate
 * @param y target y coordinate
 * @param z target z coordinate
 * @return will be moving angles for robot's joint
 */
JointAngles InverseKinematics::solve_ik_3dof(double x, double y, double z) {
  // First link length
  const double L1 = 0.425;
  // Second link length
  const double L2 = 0.392;
  joint_angles.theta1 = std::atan2(y, x);

  // The horizontal distance from the base link to the target.
  double r = std::sqrt(x*x + y*y);
  // The vertical distance from the base link to the target, adjusting for the base link's height.
  double r_v = z - 0.163;
  // The distance from the base link to the target.
  double d = std::sqrt(r*r + r_v*r_v);
  double cos_ratio = (d*d - L1*L1 - L2*L2) / (2*L1*L2);

  if (cos_ratio > 1.0) cos_ratio = 1.0;
  if (cos_ratio < -1.0) cos_ratio = -1.0;

  joint_angles.theta3 = std::acos(cos_ratio);

  // The angle between L1 and the line from the base link to the target.
  double inner_angle_1 = std::acos((L2*L2 - d*d - L1*L1) / (2*L1*d));
  // The angle of elevation to the target.
  double inner_angle_2 = std::atan2(z-0.163, r);
  joint_angles.theta2 = inner_angle_1 + inner_angle_2;
  return joint_angles;
}
