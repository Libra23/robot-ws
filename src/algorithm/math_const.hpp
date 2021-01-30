#ifndef MATH_CONST_H
#define MATH_CONST_H

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif

#define NUM_JOINT 3
#define DOF 3

enum Axis {
  X,
  Y,
  Z,
  XYZ
};

enum RotateAxis {
  ROLL,
  PITCH,
  YAW,
  RPY
};

#endif