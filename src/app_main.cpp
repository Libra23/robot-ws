#include <iostream>
#include "algorithm/kinematic.hpp"

Kinematic kinematic_;
KinematicModel model_;

extern "C" void app_main () {
    while(true) {
        std::cout << "Hello!!!!!!!!!!" << std::endl;
    }
    
    /*
    model_.xyz = {{
            {{27.5, 47.63, 0.0}},   // Yaw
            {{33.25, 0.0, 0.0}},    // Pitch1
            {{60.0, 0.0, 0.0}},     // Pitch2
            {{120.0, 0.0, 0.0}},    // Tip
            }};
    model_.axis = {{
                {{0.0, 0.0, 1.0}},      // Yaw
                {{0.0, 1.0, 0.0}},      // Pitch1
                {{0.0, 1.0, 0.0}},      // Pitch2
                {{0.0, 0.0, 0.0}},      // Tip
                }};
    model_.type = {{ROTATE, ROTATE, ROTATE, FIXED}};
    kinematic_.Config(model_, 50);
    kinematic_.SetConstant(1500.0, 0.01);   // set w & k
    */
}