#ifndef MATH_CONST_H
#define MATH_CONST_H

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif

#ifndef PI
#define PI 3.14159265358979323846264338327950288
#endif

#define GRAVITY_CONSTANT 9.8

#define NUM_WRENCH 6

#define S_TO_MS 1000
#define MS_TO_S 0.001

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