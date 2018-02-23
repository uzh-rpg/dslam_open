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
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include "rpg_common/cv_type.h"

namespace rpg_common {

// In contrast to CV's eigen conversion, this has hard checks that also compile
// in Release mode.
template <typename Type, int Rows, int Cols>
Eigen::Matrix<Type, Rows, Cols> cvToEigen(const cv::Mat& mat)
{
  CHECK_EQ(mat.rows, Rows);
  CHECK_EQ(mat.cols, Cols);
  CHECK_EQ(mat.type(), CvType<Type>::value);
  return Eigen::Map<Eigen::Matrix<Type, Rows, Cols>>(
      reinterpret_cast<Type*>(mat.data));
}

// Assumes each vector element is a column of the result.
template <typename Type>
void cvToEigen(const std::vector<cv::Mat>& mats,
               Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>* result)
{
  CHECK_NOTNULL(result);

  if (mats.empty())
  {
    result->resize(0, 0);
    return;
  }

  const int rows = mats[0].rows;
  result->resize(rows, mats.size());

  for (size_t i = 0u; i < mats.size(); ++i)
  {
    CHECK_EQ(mats[i].rows, rows);
    CHECK_EQ(mats[i].cols, 1);
    CHECK_EQ(mats[i].type(), CvType<Type>::value);
    result->col(i) = Eigen::Map<Eigen::Matrix<Type, Eigen::Dynamic, 1>>(
        reinterpret_cast<Type*>(mats[i].data), mats[i].rows);
  }
}

}  // namespace rpg_common
namespace rpg = rpg_common;
