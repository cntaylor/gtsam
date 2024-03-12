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

} //namespace gtsam