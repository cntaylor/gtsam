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

//Implement PseudoRange, but with a switchable constraint
class GTSAM_EXPORT sc_PseudoRangeFactor : public NoiseModelFactor2<Vector5, double> {
  private:
    typedef NoiseModelFactor2<Vector5,double> Base;
    Vector3 sat_pos_ = Vector3::Zero();
    double prange_meas_ = 0.;


  public:
    sc_PseudoRangeFactor(Key pos_key, Key sc_key, const double& prange_meas,
                      const Vector3& sat_pos, const SharedNoiseModel& model)
        : Base(model, pos_key, sc_key), sat_pos_(sat_pos), prange_meas_(prange_meas) {}
    
    Vector evaluateError(const Vector5& p, const double& sc, 
                         OptionalMatrixType H_p,
                         OptionalMatrixType H_sc ) const override;

};

class GTSAM_EXPORT ClockErrorFactor : public NoiseModelFactor2<Vector5, Vector5> {
  private:
    typedef NoiseModelFactor2<Vector5,Vector5> Base;
    double time_diff_ = 1.0;


  public:
    ClockErrorFactor(Key vec1, Key vec2, const double& delta_t,
                      const SharedNoiseModel& model)
        : Base(model, vec1, vec2), time_diff_(delta_t) {}
    
    Vector evaluateError(const Vector5& vec1, const Vector5& vec2, 
                         OptionalMatrixType H_vec1,
                         OptionalMatrixType H_vec2 ) const override;

};

class GTSAM_EXPORT BetweenVector5Factor : public NoiseModelFactor2<Vector5, Vector5> {
  private:
    typedef NoiseModelFactor2<Vector5, Vector5> Base;
    double time_diff_ = 1.0;

  public:
    BetweenVector5Factor(Key vec1, Key vec2, const double& delta_t,
                         const SharedNoiseModel& model)
                         : Base(model, vec1, vec2), time_diff_(delta_t) {}

    Vector evaluateError(const Vector5& vec1, const Vector5& vec2, 
                         OptionalMatrixType H_vec1,
                         OptionalMatrixType H_vec2 ) const override;
};
}