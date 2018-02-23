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

#include <fstream>

#include <Eigen/Dense>
#include <glog/logging.h>

#include "rpg_common/vector_to_eigen.h"

namespace rpg_common {

// Does the inverse of:
// Eigen::Matrix x;
// std::ofstream out("filename");
// out << x;
// I.e. loads a space-separated file into a matrix.
template <typename Type, int Rows, int Cols>
void load(const std::string& file, Eigen::Matrix<Type, Rows, Cols>* result)
{
  CHECK_NOTNULL(result);
  std::ifstream in(file);
  CHECK(in.is_open());

  std::vector<std::vector<Type>> coeffs;
  while (!in.eof())
  {
    std::string line;
    getline(in, line);

    coeffs.emplace_back();
    std::istringstream iss(line);
    while (!iss.eof())
    {
      coeffs.back().emplace_back();
      iss >> coeffs.back().back();
    }
    if (iss.fail())
    {
      coeffs.back().pop_back();
    }

    if (coeffs.back().empty())
    {
      coeffs.pop_back();
    }
  }

  vectorToEigen(coeffs, result);
}
}  // namespace rpg_common

namespace rpg = rpg_common;
