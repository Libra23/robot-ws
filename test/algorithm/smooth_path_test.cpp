#include <gtest/gtest.h>
#include "smooth_path.hpp"

//#define PLOT
#ifdef PLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

TEST(SmoothingPathTest, Check) {
    SmoothingPath<double> path;

    const int resolution = 100;
    #ifdef PLOT
    std::vector<double> x(resolution), y0(resolution);
    #endif

    path.Create(0, 0, 0, 10);
    for (int i = 0; i < resolution; i++) {
        double y = i;
        if (path.DoSmoothing()) {
            path.ModifyTarget(y);
            y = path.Get(i);
        }
        #ifdef PLOT
        x[i] = i;
        y0[i] = y;
        #endif
    }
    #ifdef PLOT
    plt::named_plot("index0", x, y0);
    plt::legend(); 
    plt::show();
    #endif
}