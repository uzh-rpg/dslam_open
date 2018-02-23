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
#include <opencv2/features2d/features2d.hpp>

#include "rpg_common/aligned.h"
#include "rpg_common/cv_type.h"

namespace rpg_common {

template <typename Type, int Rows, int Cols>
void eigenToCv(const Eigen::Matrix<Type, Rows, Cols>& eigen, cv::Mat* mat)
{
  CHECK_NOTNULL(mat)->create(eigen.rows(), eigen.cols(), CvType<Type>::value);
  Eigen::Map<Eigen::Matrix<Type, Rows, Cols>>(
      reinterpret_cast<Type*>(mat->data), eigen.rows(), eigen.cols()) = eigen;
}

// Transfers each column to an element of the result.
template <typename Type>
void eigenToCv(
    const Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>& eigen,
    std::vector<cv::Mat>* result)
{
  CHECK_NOTNULL(result)->resize(eigen.cols());

  for (size_t i = 0u; i < eigen.cols(); ++i)
  {
    (*result)[i].create(eigen.rows(), 1, CvType<Type>::value);
    Eigen::Map<Eigen::Matrix<Type, Eigen::Dynamic, 1>>(
        reinterpret_cast<Type*>((*result)[i].data), eigen.rows()) =
            eigen.col(i);
  }
}

template <typename InType, typename OutType>
void eigenToCv(const Eigen::Matrix<InType, 2, Eigen::Dynamic>& eigen,
               std::vector<cv::Point_<OutType>>* result)
{
  CHECK_NOTNULL(result)->clear();
  for (int i = 0; i < eigen.cols(); ++i)
  {
    result->emplace_back(eigen(0, i), eigen(1, i));
  }
}

template <typename InType>
void eigenToCv(const Eigen::Matrix<InType, 2, Eigen::Dynamic>& eigen,
               std::vector<cv::KeyPoint>* result)
{
  CHECK_NOTNULL(result)->clear();
  for (int i = 0; i < eigen.cols(); ++i)
  {
    result->emplace_back(eigen(0, i), eigen(1, i), 1.);
  }
}

template <typename Type, int Rows>
void eigenToCv(
    const rpg::Aligned<std::vector, Eigen::Matrix<Type, Rows, 1>>& eigen,
    cv::Mat* cv)
{
  CHECK(!eigen.empty());
  CHECK_NOTNULL(cv)->create(eigen[0].rows(), eigen.size(), CvType<Type>::value);
  const int rows = eigen[0].rows();
  for (size_t col_i = 0u; col_i < eigen.size(); ++col_i)
  {
    CHECK_EQ(eigen[col_i].rows(), rows);
    for (int row_i = 0; row_i < rows; ++row_i)
    {
      cv->at<Type>(row_i, col_i) = eigen[col_i](row_i);
    }
  }
}

template <typename Type, int Rows>
void eigenToCv(
    const Aligned<std::vector, Eigen::Matrix<Type, Rows, 1>>& eigen,
    std::vector<cv::Mat>* cv)
{
  CHECK(!eigen.empty());
  CHECK_NOTNULL(cv)->resize(eigen.size());
  for (size_t i = 0u; i < eigen.size(); ++i)
  {
    (*cv)[i].create(eigen[0].rows(), 1, CvType<Type>::value);
    Eigen::Map<Eigen::Matrix<Type, Rows, 1>>(
        reinterpret_cast<Type*>((*cv)[i].data), eigen[0].rows(), 1) = eigen[i];
  }
}

}  // namespace rpg_common
namespace rpg = rpg_common;
