/**
 * @file    PseudoRangeFactor.cpp
 * @brief   Implementation file for PseudoRangeFactor
 * @author  Clark Taylor
 * @date    Mar 2024
 **/

#include "PseudoRangeFactor.h"

using namespace std;

namespace gtsam {

    namespace prange{ 
        double time_divider=1E6;
        double c = 2.99792458E8; // m/s, speed of light
        double c_small = c/time_divider; // speed of light in smaller time units (to make solving more numerically stable)
        double sc_epsilon = 1E-5; // Used to make square root derivatives not blow up
    }

Vector PseudoRangeFactor::evaluateError(const Vector5& p, 
                                        OptionalMatrixType H) const {
    Vector3 diff_loc = p.head<3>() - sat_pos_;
    double prange_error = diff_loc.norm() + p[3]*prange::c_small - prange_meas_;
    auto normed_diff = diff_loc/diff_loc.norm();
    Vector1 err;
    err[0] = prange_error;

    if (H) {
        Matrix tmp(1, 5);
        for (int i=0; i<3; i++) 
            tmp(0,i) = normed_diff[i];
        tmp(0,3) = prange::c_small;
        tmp(0,4) = 0.0;
        // std::cout << "tmp created successfully" << std::endl;
        (*H) = tmp;
    }
    return err;
}

Vector sc_PseudoRangeFactor::evaluateError(const Vector5& p, const double& sc,
                                        OptionalMatrixType H_p,
                                        OptionalMatrixType H_sc) const {
    Vector3 diff_loc = p.head<3>() - sat_pos_;
    double uw_error = diff_loc.norm() + p[3]*prange::c_small - prange_meas_; // unweighted error
    auto normed_diff = diff_loc/diff_loc.norm();

    // I don't know if this is needed or not, but if the switch goes negative, that would
    // throw off the math, so just make sure it doesnt... (while getting the value)
    double orig_switch = std::fmax(0,sc);
    // I do sqrt because I want the switch to scale the squared error, not (possibly negative) error
    double local_switch = std::sqrt(orig_switch);
    double error = uw_error * local_switch;

    if (H_sc) {
        // Maybe I don't need to, but I am worried about the / orig_switch value (the correct value)
        // being numerically stable as the switch value approaches 0.  So, use the sc_epsilon here.
        Matrix tmp(1,1);
        tmp(0,0) = uw_error * 0.5/(local_switch + prange::sc_epsilon);
        (*H_sc) = tmp;
    }

    if (H_p) {
        Matrix tmp(1, 5);
        for (int i=0; i<3; i++) 
            tmp(0,i) = normed_diff[i];
        tmp(0,3) = prange::c_small;
        tmp(0,4) = 0.0;
        // std::cout << "tmp created successfully" << std::endl;
        (*H_p) = tmp * local_switch;
    }
    return (Vector1() << error).finished();
}

Vector ClockErrorFactor::evaluateError(const Vector5& vec1, const Vector5& vec2, 
                                        OptionalMatrixType H_vec1,
                                        OptionalMatrixType H_vec2) const {
    double time_error = vec2[3] - vec1[3] - vec1[4]*time_diff_;
    double rate_error = vec2[4] - vec1[4];
    if (H_vec1) {
        Matrix tmp = Matrix::Zero(2, 5);
        tmp(0,3) = -1.0;
        tmp(0,4) = -time_diff_;
        tmp(1,4) = -1.0;
        (*H_vec1) = tmp;
    }
    if (H_vec2) {
        Matrix tmp = Matrix::Zero(2,5);
        tmp(0,3) = 1.0;
        tmp(1,4) = 1.0;
        (*H_vec2) = tmp;
    }
    return (Vector2() << time_error, rate_error).finished();
}

Vector BetweenVector5Factor::evaluateError(const Vector5& vec1, const Vector5& vec2, 
                                        OptionalMatrixType H_vec1,
                                        OptionalMatrixType H_vec2) const {
    Vector pos_diff = vec2.head<3>() - vec1.head<3>();
    double time_error = vec2[3] - vec1[3] - vec1[4]*time_diff_;
    double rate_error = vec2[4] - vec1[4];
    if (H_vec1) {
        Matrix tmp = Matrix::Zero(5, 5);
        for (int i=0;i<3;i++) { //position error stuff
            tmp(i,i) = -1.0;
        }
        tmp(3,3) = -1.0;
        tmp(3,4) = -time_diff_;
        tmp(4,4) = -1.0;
        (*H_vec1) = tmp;
    }
    if (H_vec2) {
        Matrix tmp = Matrix::Zero(5,5);
        for (int i=0;i<3;i++) { //position error stuff
            tmp(i,i) = 1.0;
        }
        tmp(3,3) = 1.0;
        tmp(4,4) = 1.0;
        (*H_vec2) = tmp;
    }
    return (Vector5() << pos_diff,time_error, rate_error).finished();
}
} //namespace gtsam