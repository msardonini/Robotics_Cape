#include "flyMS/imu/base.h"

bool ImuBase::does_imu_obj_exist = false;

ImuBase::ImuBase() {
  if (ImuBase::does_imu_obj_exist) {
    throw std::invalid_argument("User tried to instantiate more than one ImuBase");
  }
  ImuBase::does_imu_obj_exist = true;
}

void ImuBase::UnwrapYaw(state_t *imu_state_body) {
  imu_state_body->euler(2) += num_wraps_ * 2 * M_PI;
  if (fabs(imu_state_body->euler[2] - imu_state_body->eulerPrevious[2])  > 5) {
    if (imu_state_body->euler[2] > imu_state_body->eulerPrevious[2]) {
      num_wraps_--;
      imu_state_body->euler[2] -= 2 * M_PI;
    } else {
      num_wraps_++;
      imu_state_body->euler[2] += 2 * M_PI;
    }
  }
}

