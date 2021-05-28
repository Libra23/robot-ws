#include <cmath>

template <typename T>
SmoothingPath<T>::SmoothingPath() {

}

template <typename T>
void SmoothingPath<T>::Create(const T& start, const T& target, double start_time, double smoothing_time) {
    target_ = target;
    start_ = start;
    start_time_ = start_time;
    smoothing_time_ = smoothing_time;
    do_smoothing_ = true;
}

template <typename T>
T SmoothingPath<T>::Get(double time) {
    double t = (time - start_time_) / smoothing_time_;
    if (t <= 0) {
        return start_;
    } else if (t >= 1.0) {
        do_smoothing_ = false;
        return target_;
    } else {
        T amp = (target_ - start_);
        return amp * (6 * pow(t, 5) - 15 * pow(t, 4) + 10 * pow(t, 3)) + start_;
    }
}

template <typename T>
void SmoothingPath<T>::ModifyTarget(const T& target) {
    target_ = target;
}

template <typename T>
bool SmoothingPath<T>::DoSmoothing() {
    return do_smoothing_;
}