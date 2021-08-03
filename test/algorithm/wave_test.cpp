#include <gtest/gtest.h>
#include "wave.hpp"

#define PLOT
#ifdef PLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

/**
 * @test check wave
 */
TEST(WaveTest, CheckWavePlot) {
    WaveForm form;
    form.amplitude = 1.0;
    form.base = 1.0;
    form.frequency = 1.0;
    form.phase = 0.0;

    Wave const_wave;
    form.type = WaveType::CONST;
    const_wave.Config(form);

    Wave sin_wave;
    form.type = WaveType::SIN;
    sin_wave.Config(form);

    Wave rect_wave;
    form.type = WaveType::RECTANGLE;
    rect_wave.Config(form);

    Wave tri_wave;
    form.type = WaveType::TRIANGLE;
    tri_wave.Config(form);

    const double resolution = 1000;
    std::vector<double> time(resolution), y0(resolution), y1(resolution), y2(resolution), y3(resolution);

    for (int i = 0; i < resolution; i++) {
        const double t = 1 / form.frequency / resolution * i;
        time[i] = t;
        y0[i] = const_wave.Update(t);
        y1[i] = sin_wave.Update(t);
        y2[i] = rect_wave.Update(t);
        y3[i] = tri_wave.Update(t);
    }

    #ifdef PLOT
    plt::named_plot("index0", time, y0);
    plt::legend();
    plt::named_plot("index1", time, y1);
    plt::legend();
    plt::named_plot("index2", time, y2);
    plt::legend();
    plt::named_plot("index3", time, y3);
    plt::legend();

    plt::show();
    #endif
}