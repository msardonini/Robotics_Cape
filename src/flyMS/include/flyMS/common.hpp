/**
 * @file common.hpp
 * @brief Common structs used within the flyMS flight program
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#ifndef SRC_FLYMS_INCLUDE_FLYMS_COMMON_H_
#define SRC_FLYMS_INCLUDE_FLYMS_COMMON_H_

#include "Eigen/Dense"
#include "filter.h"

constexpr float MICROTESLA_TO_GAUSSf = 0.01f;
constexpr double MICROTESLA_TO_GAUSS = 0.01;
constexpr float R2Df =  57.295779513f;
constexpr float D2Rf =  0.0174532925199f;
constexpr double R2D =  57.295779513;
constexpr double D2R =  0.0174532925199;
constexpr double DT = 0.01;









#endif  // SRC_FLYMS_INCLUDE_FLYMS_COMMON_H_
