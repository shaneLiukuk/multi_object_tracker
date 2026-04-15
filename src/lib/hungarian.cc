#include "od_fusion/lib/hungarian.h"
#include <cmath>
#include <iostream>

namespace perception {
namespace fusion {

const float Hungarian::kInf = 1e6f;

void Hungarian::Solve(const Eigen::MatrixXf& cost_matrix,
                      Eigen::MatrixXi* assignment) {
  int n_rows = static_cast<int>(cost_matrix.rows());
  int n_cols = static_cast<int>(cost_matrix.cols());

  if (assignment == nullptr) {
    return;
  }
  assignment->setZero(n_rows, n_cols);

  if (n_rows == 0 || n_cols == 0) {
    return;
  }

  int nOfRows = n_rows;
  int nOfCols = n_cols;

  std::vector<double> dist_matrix_in(n_rows * n_cols);
  for (int i = 0; i < n_rows; ++i) {
    for (int j = 0; j < n_cols; ++j) {
      dist_matrix_in[i + n_rows * j] = static_cast<double>(cost_matrix(i, j));
    }
  }

  std::vector<int> assign(n_rows, -1);
  double cost = 0.0;

  std::vector<double> dist_matrix(n_rows * n_cols);
  for (size_t i = 0; i < dist_matrix_in.size(); ++i) {
    dist_matrix[i] = dist_matrix_in[i];
  }

  std::vector<bool> covered_columns(n_cols, false);
  std::vector<bool> covered_rows(n_rows, false);
  std::vector<bool> star_matrix(n_rows * n_cols, false);
  std::vector<bool> prime_matrix(n_rows * n_cols, false);
  std::vector<bool> new_star_matrix(n_rows * n_cols, false);

  int min_dim = std::min(n_rows, n_cols);

  if (n_rows <= n_cols) {
    for (int row = 0; row < n_rows; ++row) {
      double min_value = dist_matrix[row];
      for (int col = 1; col < n_cols; ++col) {
        double value = dist_matrix[row + n_rows * col];
        if (value < min_value) {
          min_value = value;
        }
      }
      for (int col = 0; col < n_cols; ++col) {
        dist_matrix[row + n_rows * col] -= min_value;
      }
    }

    for (int row = 0; row < n_rows; ++row) {
      for (int col = 0; col < n_cols; ++col) {
        if (std::fabs(dist_matrix[row + n_rows * col]) < 1e-9 &&
            !covered_columns[col]) {
          star_matrix[row + n_rows * col] = true;
          covered_columns[col] = true;
          break;
        }
      }
    }
  }

  int* assignment_arr = assign.data();

  while (true) {
    int n_covered_columns = 0;
    for (int col = 0; col < n_cols; ++col) {
      if (covered_columns[col]) {
        n_covered_columns++;
      }
    }

    if (n_covered_columns >= min_dim || n_rows == 0) {
      break;
    }

    int row = -1;
    int col = -1;
    bool found = false;

    for (int r = 0; r < n_rows && !found; ++r) {
      if (covered_rows[r]) continue;
      for (int c = 0; c < n_cols; ++c) {
        if (covered_columns[c]) continue;
        if (std::fabs(dist_matrix[r + n_rows * c]) < 1e-9) {
          prime_matrix[r + n_rows * c] = true;
          int star_col = -1;
          for (int sc = 0; sc < n_cols; ++sc) {
            if (star_matrix[r + n_rows * sc]) {
              star_col = sc;
              break;
            }
          }
          if (star_col == -1) {
            row = r;
            col = c;
            found = true;
            break;
          } else {
            covered_rows[r] = true;
            covered_columns[star_col] = false;
          }
        }
      }
    }

    if (!found) {
      double h = kInf;
      for (int r = 0; r < n_rows; ++r) {
        if (covered_rows[r]) continue;
        for (int c = 0; c < n_cols; ++c) {
          if (covered_columns[c]) continue;
          double value = dist_matrix[r + n_rows * c];
          if (value < h) {
            h = value;
          }
        }
      }

      for (int r = 0; r < n_rows; ++r) {
        if (covered_rows[r]) {
          for (int c = 0; c < n_cols; ++c) {
            dist_matrix[r + n_rows * c] += h;
          }
        }
      }
      for (int c = 0; c < n_cols; ++c) {
        if (!covered_columns[c]) {
          for (int r = 0; r < n_rows; ++r) {
            dist_matrix[r + n_rows * c] -= h;
          }
        }
      }
    } else {
      new_star_matrix[row + n_rows * col] = true;

      int star_row = -1;
      for (int r = 0; r < n_rows; ++r) {
        if (star_matrix[r + n_rows * col]) {
          star_row = r;
          break;
        }
      }

      while (star_row >= 0) {
        new_star_matrix[star_row + n_rows * col] = false;

        int prime_col = -1;
        for (int c = 0; c < n_cols; ++c) {
          if (prime_matrix[star_row + n_rows * c]) {
            prime_col = c;
            break;
          }
        }

        new_star_matrix[star_row + n_rows * prime_col] = true;

        star_row = -1;
        for (int r = 0; r < n_rows; ++r) {
          if (star_matrix[r + n_rows * prime_col]) {
            star_row = r;
            break;
          }
        }
      }

      for (int i = 0; i < n_rows * n_cols; ++i) {
        prime_matrix[i] = false;
        star_matrix[i] = new_star_matrix[i];
      }
      for (int r = 0; r < n_rows; ++r) {
        covered_rows[r] = false;
      }

      for (int c = 0; c < n_cols; ++c) {
        for (int r = 0; r < n_rows; ++r) {
          if (star_matrix[r + n_rows * c]) {
            covered_columns[c] = true;
            break;
          }
        }
      }
    }
  }

  for (int r = 0; r < n_rows; ++r) {
    for (int c = 0; c < n_cols; ++c) {
      if (star_matrix[r + n_rows * c]) {
        assign[r] = c;
        cost += dist_matrix_in[r + n_rows * c];
        break;
      }
    }
  }

  for (int i = 0; i < n_rows; ++i) {
    if (assign[i] >= 0 && assign[i] < n_cols) {
      (*assignment)(i, assign[i]) = 1;
    }
  }
}

}  // namespace fusion
}  // namespace perception
