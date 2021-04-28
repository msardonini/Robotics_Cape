#ifndef SRC_FLYMS_INCLUDE_FLYMS_TYPES_VIO_H_
#define SRC_FLYMS_INCLUDE_FLYMS_TYPES_VIO_H_

#include "Eigen/Dense"

struct VioData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  Eigen::Quaternionf quat;
};


#endif  //SRC_FLYMS_INCLUDE_FLYMS_TYPES_VIO_H_
