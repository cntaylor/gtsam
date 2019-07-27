/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testBearingRangeFactor.cpp
 *  @brief Unit tests for BearingRangeFactor Class
 *  @author Frank Dellaert
 *  @date July 2015
 */

#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/base/serialization.h>
#include <gtsam/base/serializationTestHelpers.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

Key poseKey_tbrf(1);
Key pointKey_tbrf(2);

typedef BearingRangeFactor<Pose2, Point2> BearingRangeFactor2D;
static SharedNoiseModel model2D_tbrf(noiseModel::Isotropic::Sigma(2, 0.5));
BearingRangeFactor2D factor2D_tbrf(poseKey_tbrf, pointKey_tbrf, 1, 2, model2D_tbrf);

typedef BearingRangeFactor<Pose3, Point3> BearingRangeFactor3D;
static SharedNoiseModel model3D_tbrf(noiseModel::Isotropic::Sigma(3, 0.5));
BearingRangeFactor3D factor3D_tbrf(poseKey_tbrf, pointKey_tbrf,
                              Pose3().bearing(Point3(1, 0, 0)), 1, model3D_tbrf);

/* ************************************************************************* */
// Export Noisemodels
// See http://www.boost.org/doc/libs/1_32_0/libs/serialization/doc/special.html
BOOST_CLASS_EXPORT(gtsam::noiseModel::Isotropic);

/* ************************************************************************* */
TEST(BearingRangeFactor, Serialization2D) {
  EXPECT(serializationTestHelpers::equalsObj(factor2D_tbrf));
  EXPECT(serializationTestHelpers::equalsXML(factor2D_tbrf));
  EXPECT(serializationTestHelpers::equalsBinary(factor2D_tbrf));
}

/* ************************************************************************* */
TEST(BearingRangeFactor, 2D) {
  // Serialize the factor
  std::string serialized = serializeXML(factor2D_tbrf);

  // And de-serialize it
  BearingRangeFactor2D factor;
  deserializeXML(serialized, factor);

  // Set the linearization point
  Values values;
  values.insert(poseKey_tbrf, Pose2(1.0, 2.0, 0.57));
  values.insert(pointKey_tbrf, Point2(-4.0, 11.0));

  EXPECT_CORRECT_EXPRESSION_JACOBIANS(factor.expression(poseKey_tbrf, pointKey_tbrf),
                                      values, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(BearingRangeFactor, Serialization3D) {
  EXPECT(serializationTestHelpers::equalsObj(factor3D_tbrf));
  EXPECT(serializationTestHelpers::equalsXML(factor3D_tbrf));
  EXPECT(serializationTestHelpers::equalsBinary(factor3D_tbrf));
}

/* ************************************************************************* */
// TODO(frank): this test is disabled (for now) because the macros below are
// incompatible with the Unit3 localCoordinates. See testBearingFactor...
//TEST(BearingRangeFactor, 3D) {
//  // Serialize the factor
//  std::string serialized = serializeXML(factor3D_tbrf);
//
//  // And de-serialize it
//  BearingRangeFactor3D factor;
//  deserializeXML(serialized, factor);
//
//  // Set the linearization point
//  Values values;
//  values.insert(poseKey_tbrf, Pose3());
//  values.insert(pointKey_tbrf, Point3(1, 0, 0));
//
//  EXPECT_CORRECT_EXPRESSION_JACOBIANS(factor.expression(poseKey_tbrf, pointKey_tbrf),
//                                      values, 1e-7, 1e-5);
//  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
//}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
