#ifndef MOTION_GENERATOR_H
#define MOTION_GENERATOR_H

#include "algorithm/math_utility.hpp"
#include "control_data/mainte_data.hpp"
#include <vector>

enum State {
    INITIALIZE,
    STOP,
    RUN
};

class GeneratorBase {
    public:
    GeneratorBase();
    virtual ~GeneratorBase();
    virtual void Config(const Reference& reference);
    virtual void Update(double time, const VectorXd& q_ref_pre, VectorXd& q_ref);
    void StartRequest();
    void StopRequest();
    void CompleteConfig();
    protected:
    State state_;
};

class FKGenerator : public GeneratorBase {
    public:
    FKGenerator(uint8_t num_of_joints);
    void Config(const Reference& reference) override;
    void Update(double time, const VectorXd& q_ref_pre, VectorXd& q_ref) override;
    private:
    std::vector<Wave> wave_;
};

#endif