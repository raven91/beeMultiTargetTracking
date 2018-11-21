//
// Created by Nikita Kruk on 14.06.18.
//

#ifndef BEEMULTITARGETTRACKING_DEFINITIONS_HPP
#define BEEMULTITARGETTRACKING_DEFINITIONS_HPP

#include <cmath>

typedef double Real;
typedef long CostInt;

const int kNumOfStateVars = 4;          // x,y,v_x,v_y
const int kNumOfDetectionVars = 2;      // x,y
const int kNumOfExtractedFeatures = 6;  // x,y,v_x,v_y,width,height

/*
 * Transform numerator into [0, denominator)
 */
inline Real WrappingModulo(Real numerator, Real denominator)
{
  return numerator - denominator * std::floor(numerator / denominator);
}

#endif //BEEMULTITARGETTRACKING_DEFINITIONS_HPP
