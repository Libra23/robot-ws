#ifndef ROBOT_CONST_H
#define ROBOT_CONST_H


// Robot Type
#define QUAD_DIAGONAL   (1)
#define QUAD_PUPPER     (2)
#define SCRATCH_PENGUIN (3)

//#define ROBOT_TYPE QUAD_DIAGONAL
#define ROBOT_TYPE QUAD_PUPPER
//#define ROBOT_TYPE SCRATCH_PENGUIN

inline int GetRobotType() {
    return ROBOT_TYPE;
}

inline bool IsQuadDiagonal() {
    if (GetRobotType() == QUAD_DIAGONAL) {
        return true;
    } else {
        return false;
    }
}

inline bool IsQuadPupper() {
    if (GetRobotType() == QUAD_PUPPER) {
        return true;
    } else {
        return false;
    }
}

inline bool IsScratchPenguin() {
    if (GetRobotType() == SCRATCH_PENGUIN) {
        return true;
    } else {
        return false;
    }
}

// NUM_JOINT define
#if ROBOT_TYPE == QUAD_DIAGONAL
    #define NUM_JOINT 3
#elif ROBOT_TYPE == QUAD_PUPPER
    #define NUM_JOINT 3
#elif ROBOT_TYPE == SCRATCH_PENGUIN
    #define NUM_JOINT 4
#endif

// NUM_ARM define
#if ROBOT_TYPE == QUAD_DIAGONAL
enum ArmId {
    LEFT_FRONT,
    LEFT_BACK,
    RIGHT_FRONT,
    RIGHT_BACK,
    NUM_ARM
};
#elif ROBOT_TYPE == QUAD_PUPPER
enum ArmId {
    LEFT_FRONT,
    LEFT_BACK,
    RIGHT_FRONT,
    RIGHT_BACK,
    NUM_ARM
};
#elif ROBOT_TYPE == SCRATCH_PENGUIN
enum ArmId {
    RIGHT,
    LEFT,
    NUM_ARM
};
#endif

#endif