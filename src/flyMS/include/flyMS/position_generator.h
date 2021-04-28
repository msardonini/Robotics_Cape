#ifndef SRC_FLYMS_INCLUDE_FLYMS_POSITION_GENERATOR_H_
#define SRC_FLYMS_INCLUDE_FLYMS_POSITION_GENERATOR_H_

#include "yaml-cpp/yaml.h"
#include "Eigen/Dense"


// TODO(msardonini): make this read in a set of waypoints and just follow that
class PositionGenerator {
 public:
  explicit PositionGenerator() {
    path_ = PATH_TYPE::LINE_X;
  }

  enum class PATH_TYPE {
    LINE_X,
    LINE_Y,
    BOX_XY,
    BOS_YX
  };

  void ResetCounter() {
    counter_ = 0;
  }

  int GetPosition(Eigen::Vector3f *output_pos) {
    Eigen::Vector3f pos;
    switch (path_) {
      case PATH_TYPE::LINE_X: {
        // For the first 10 seconds, remain stationary
        if (counter_ < 10 * 100) {
          pos = Eigen::Vector3f::Zero();
        } else if (counter_ < 30 * 100) {  // spend 20 seconds moving forward in X
          float x_pos = 15. * (static_cast<float>(counter_) - 10. * 100.) / (20. * 100.);
          pos << x_pos, 0.0f, 0.0f;
        } else if (counter_ < 50 * 100) {  // spend the next 20 moving in -Y
          float y_pos = -15. * (static_cast<float>(counter_) - 30. * 100.) / (20. * 100.);
          pos << 15, y_pos, 0.0f;
        } else if (counter_ < 70 * 100) {  // spend the next 20 seconds moving back in X
          float x_pos = 15. - 15. * (static_cast<float>(counter_) - 50. * 100.) / (20. * 100.);
          pos << x_pos, -15.0f, 0.0f;
        } else if (counter_ < 90 * 100) {  // spend the next 20 seconds moving back in X
          float y_pos = -15. + 15. * (static_cast<float>(counter_) - 70. * 100.) / (20. * 100.);
          pos << 0, y_pos, 0.0f;
        } else if (counter_ < 110 * 100) {  // spend the next 20 seconds moving back in X
          pos = Eigen::Vector3f::Zero();
        }
      }
    }
    *output_pos = pos;
    counter_++;
    return 0;
  }

 private:
  unsigned int counter_ = 0;
  PATH_TYPE path_;
};


#endif  // SRC_FLYMS_INCLUDE_FLYMS_POSITION_GENERATOR_H_