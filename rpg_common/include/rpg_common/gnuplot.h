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

#include <Eigen/Dense>

#include "rpg_common/aligned.h"

namespace rpg_common {

// Launches an instance of Gnuplot to talk to. To plot data:
//
// rpg::Gnuplot plot;
// plot << "plot '-' w p";
// plot << your_matrix;
//
// See gnuplot documentation for commands.
//
// Needs `sudo apt-get install gnuplot-x11`
class Gnuplot
{
public:
  explicit Gnuplot(const bool persist = false);

  // Adds new line at end!
  void operator <<(const std::string& string);

  template <int Rows, int Cols>
  void operator <<(const Eigen::Matrix<double, Rows, Cols>& data)
  {
    for (int row_i = 0; row_i < data.rows(); ++row_i)
    {
      for (int col_i = 0; col_i < data.cols(); ++col_i)
      {
        fprintf(pipe_, "%lf ", data(row_i, col_i));
      }
      fprintf(pipe_, "\n");
    }
    fprintf(pipe_, "e\n");
    fflush(pipe_);
  }

  template <int Cols>
  void operator <<(
      const Aligned<std::vector, Eigen::Matrix<double, 1, Cols>>& data)
  {
    for (const Eigen::Matrix<double, 1, Cols>& row : data)
    {
      for (int col_i = 0; col_i < row.cols(); ++col_i)
      {
        fprintf(pipe_, "%lf ", row(col_i));
      }
      fprintf(pipe_, "\n");
    }
    fprintf(pipe_, "e\n");
    fflush(pipe_);
  }

  void feedAsXy(const std::vector<double>& x, const std::vector<double>& y);

private:
  FILE* pipe_;
};

}  // namespace rpg_common
namespace rpg = rpg_common;
