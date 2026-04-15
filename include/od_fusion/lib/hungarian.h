#pragma once

#include <vector>
#include <Eigen/Dense>

namespace perception {
namespace fusion {

class Hungarian {
 public:
  static void Solve(const Eigen::MatrixXf& cost_matrix,
                    Eigen::MatrixXi* assignment);

 private:
  static const float kInf;
};

}  // namespace fusion
}  // namespace perception
