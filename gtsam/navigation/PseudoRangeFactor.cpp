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

    if (H) (*H) = (Matrix(1, 5) << normed_diff, prange::c_small, 0.0).finished();
    return (Vector(1) << prange_error).finished();
}

} //namespace gtsam