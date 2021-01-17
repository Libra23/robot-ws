template<typename T>
class SmoothingPath {
    public:
    SmoothingPath();
    void Create(const T& target, const T& start, double start_time, double smoothing_time);
    T Update(double time);
    void ModifyTarget(const T& target);

    private:
    T target_;
    T start_;
    double start_time_;
    double smoothing_time_;
};