// Copyright (C) 2017-2018 Titus Cieslewski, RPG, University of Zurich,
//   Switzerland
//   You can contact the author at <titus at ifi dot uzh dot ch>
// Copyright (C) 2017-2018 Davide Scaramuzza, RPG, University of Zurich, 
//   Switzerland
//
// This file is part of dslam_open.
//
// dslam_open is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// dslam_open is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with dslam_open. If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <vector>
#include <type_traits>

#include <Eigen/Dense>
#include <glog/logging.h>

namespace rpg_common {

// for a single vector:
//    vectorToEigen({in}, out);
// This function uses row-major: each vector stores a row of the matrix.
template <typename Type, int Rows, int Cols>
void vectorToEigen(const std::vector<std::vector<Type>>& in,
                   Eigen::Matrix<Type, Rows, Cols>* out)
{
  CHECK_NOTNULL(out);
  CHECK(!in.empty());

  CHECK(Rows == Eigen::Dynamic || in.size() == Rows)
      << "Row dimension is not consistent.";
  CHECK(Cols == Eigen::Dynamic || in[0].size() == Cols)
      << "Column dimension is not consistent.";
  for (size_t i = 0; i < in.size() - 1; i++)
  {
    CHECK_EQ(in[i].size(), in[i+1].size());
  }
  out->resize(in.size(), in[0].size());

  for (size_t row_i = 0u; row_i < in.size(); ++row_i)
  {
    for (size_t col_i = 0u; col_i < in[row_i].size(); ++col_i)
    {
      (*out)(row_i, col_i) = in[row_i][col_i];
    }
  }
}

}

namespace rpg = rpg_common;
