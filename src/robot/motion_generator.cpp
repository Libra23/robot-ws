#include "motion_generator.hpp"

/**
 * @class GeneratorBase
 */
GeneratorBase::GeneratorBase()
    : state_(State::INITIALIZE) {}

GeneratorBase::~GeneratorBase() {}

void GeneratorBase::Config(const Reference& reference) {}

void GeneratorBase::Update(double time, const VectorXd& q_ref_pre, VectorXd& q_ref) {}

void GeneratorBase::StartRequest() {
    if (state_ == State::STOP) {
        state_ = State::RUN;
    }
}

void GeneratorBase::StopRequest() {
    if ((state_ == State::INITIALIZE) || (state_ == State::RUN)) {
        state_ = State::STOP;
    }
}

void GeneratorBase::CompleteConfig() {
    if (state_ == State::INITIALIZE) {
        state_ = State::STOP;
    }
}

/**
 * @class FKGenerator
 */
FKGenerator::FKGenerator(uint8_t num_of_joints)
    :   GeneratorBase(),
        wave_(num_of_joints) {}

FKGenerator::~FKGenerator() {}

void FKGenerator::Config(const Reference& reference) {
    for (size_t i = 0; i < wave_.size(); i++) {
        wave_[i].Config(reference.fk[i].type, reference.fk[i].amplitude * DEG_TO_RAD, reference.fk[i].base * DEG_TO_RAD, reference.fk[i].frequency, reference.fk[i].phase * DEG_TO_RAD);
    }
}

void FKGenerator::Update(double time, const VectorXd& q_ref_pre, VectorXd& q_ref) {
    if (state_ == State::RUN) {
        for (size_t i = 0; i < wave_.size(); i++) {
            q_ref[i] = wave_[i].Update(time);
        }
    } else {
        q_ref = q_ref_pre;
    }

}

/**
 * @class IKGenerator
 */
IKGenerator::IKGenerator(const std::shared_ptr<Arm>& p_arm)
    : p_arm_(p_arm) {}

void IKGenerator::Config(const Reference& reference) {
    for (size_t i = 0; i < wave_.size(); i++) {
        wave_[i].Config(reference.fk[i].type, reference.fk[i].amplitude, reference.fk[i].base, reference.fk[i].frequency, reference.fk[i].phase);
    }
}

void IKGenerator::Update(double time, const VectorXd& q_ref_pre, VectorXd& q_ref) {
    if (state_ == State::RUN) {
        for (size_t i = 0; i < wave_.size(); i++) {
            q_ref[i] = wave_[i].Update(time);
        }
    } else {
        q_ref = q_ref_pre;
    }
}