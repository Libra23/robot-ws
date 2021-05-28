#ifndef SMOOTH_PATH_H
#define SMOOTH_PATH_H

template<typename T>
class SmoothingPath {
    public:
    SmoothingPath();
    void Create(const T& start, const T& target, double start_time, double smoothing_time);
    T Get(double time);
    void ModifyTarget(const T& target);
    bool DoSmoothing();

    private:
    T target_;
    T start_;
    double start_time_;
    double smoothing_time_;
    bool do_smoothing_;
};

#include "smooth_path.inl"

#endif