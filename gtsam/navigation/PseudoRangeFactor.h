/**
 * @file    PseudoRangeFactor.h
 * @brief   pseudo range factor for raw GPS measurements (but already corrected and with known satellite in ECEF)
 * @author  Clark Taylor
 * @date    Mar 2024
 **/

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/* In the future, should really make this more robust, but for now
just assume the values are a vector5, where the first 3 are the ECEF location
and the fourth is the clock error */

class GTSAM_EXPORT PseudoRangeFactor : public NoiseModelFactor1<Vector5> {
  private:
    typedef NoiseModelFactor1<Vector5> Base;
    Vector3 sat_pos_ = Vector3::Zero();
    double prange_meas_ = 0.;

  public:
    PseudoRangeFactor(Key key, const double& prange_meas,
                      const Vector3& sat_pos, const SharedNoiseModel& model)
        : Base(model, key), sat_pos_(sat_pos), prange_meas_(prange_meas) {}
    
    Vector evaluateError(const Vector5& p, 
                         OptionalMatrixType H ) const override;


};
}