#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "pose_estimator.hpp"
//#include <matplotlib/matplotlibcpp.h>

constexpr double TOLERANCE = 1e-6;

class PoseEstimatorTest : public ::testing::Test {
    protected:
    PoseEstimatorTest() {

    }
    PoseEstimator estimator_;
};

///////////////////////////////////////
// Check linear kalman filter component
///////////////////////////////////////
TEST_F(PoseEstimatorTest, CheckLinearEquation) {

}