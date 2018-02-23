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
#include <numeric>

#include <Eigen/Dense>
#include <glog/logging.h>

#include "rpg_common/aligned.h"

namespace rpg_common {

// =====================
// SELECT BY BOOL VECTOR
// =====================

template <typename Type>
inline std::vector<Type> select(
    const std::vector<Type>& full,
    const std::vector<bool>& filter)
{
  CHECK_EQ(full.size(), filter.size());
  std::vector<Type> result;

  size_t i = 0u;
  for (const Type& object : full)
  {
    if (filter[i])
    {
      result.emplace_back(object);
    }
    ++i;
  }

  return result;
}

template <typename Scalar, int Rows>
inline Aligned<std::vector, Eigen::Matrix<Scalar, Rows, 1>> select(
    const Aligned<std::vector, Eigen::Matrix<Scalar, Rows, 1>>& full,
    const std::vector<bool>& filter)
{
  CHECK_EQ(full.size(), filter.size());
  Aligned<std::vector, Eigen::Matrix<Scalar, Rows, 1>> result;

  size_t i = 0u;
  for (const Eigen::Matrix<Scalar, Rows, 1>& object : full)
  {
    if (filter[i])
    {
      result.emplace_back(object);
    }
    ++i;
  }

  return result;
}

inline std::vector<size_t> select(
    const std::vector<bool>& filter)
{
  std::vector<size_t> indices(filter.size());
  std::iota(indices.begin(), indices.end(), 0u);
  return select(indices, filter);
}

template <typename Scalar, int Rows>
inline Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> select(
    const Eigen::Matrix<Scalar, Rows, Eigen::Dynamic>& full,
    const std::vector<bool>& filter)
{
  CHECK_EQ(full.cols(), filter.size());
  const int result_size = std::count(filter.begin(), filter.end(), true);
  Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> result(full.rows(), result_size);

  int result_i = 0;
  for (int i = 0; i < filter.size(); ++i)
  {
    if (filter[i]) {
      result.col(result_i) = full.col(i);
      ++result_i;
    }
  }

  return result;
}

template <typename Scalar, int Cols>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, Cols> select(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Cols>& full,
    const std::vector<bool>& filter)
{
  CHECK_EQ(full.rows(), filter.size());
  const int result_size = std::count(filter.begin(), filter.end(), true);
  Eigen::Matrix<Scalar, Eigen::Dynamic, Cols> result(result_size, full.cols());

  int result_i = 0;
  for (int i = 0; i < filter.size(); ++i)
  {
    if (filter[i]) {
      result.row(result_i) = full.row(i);
      ++result_i;
    }
  }

  return result;
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> selectCols(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& full,
    const std::vector<bool>& filter)
{
  CHECK_EQ(full.cols(), filter.size());
  const int result_size = std::count(filter.begin(), filter.end(), true);
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> result(
      full.rows(), result_size);

  int result_i = 0;
  for (int i = 0; i < filter.size(); ++i)
  {
    if (filter[i]) {
      result.col(result_i) = full.col(i);
      ++result_i;
    }
  }

  return result;
}

// =================
// SELECT BY INDICES
// =================

template <typename Type>
inline std::vector<Type> select(
    const std::vector<Type>& full,
    const Eigen::VectorXi& indices)
{
  CHECK_LT(indices.maxCoeff(), full.size());
  CHECK_GE(indices.minCoeff(), 0);
  std::vector<Type> result(indices.size());

  for (int i = 0; i < indices.size(); ++i)
  {
    result[i] = full[indices[i]];
  }

  return result;
}

template <typename Type>
inline std::vector<Type> select(
    const std::vector<Type>& full,
    const std::vector<size_t>& indices)
{
  std::vector<Type> result;
  result.reserve(indices.size());

  for (const size_t index : indices)
  {
    CHECK_LT(index, full.size());
    result.push_back(full[index]);
  }

  return result;
}

template <typename Scalar, int Rows>
inline Eigen::Matrix<Scalar, Rows, 1> select(
    const Eigen::Matrix<Scalar, Rows, 1>& full,
    const std::vector<size_t>& indices)
{
  Eigen::Matrix<Scalar, Rows, 1> result(indices.size(), 1);
  for (int i = 0; i < indices.size(); ++i)
  {
    CHECK_LT(indices[i], full.size());
    result(i) = full(indices[i]);
  }

  return result;
}

template <typename Scalar, int Rows>
inline Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> select(
    const Eigen::Matrix<Scalar, Rows, Eigen::Dynamic>& full,
    const std::vector<size_t>& indices)
    {
  Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> result(
      full.rows(), indices.size());
  for (int i = 0; i < indices.size(); ++i)
  {
    CHECK_LT(indices[i], full.cols());
    result.col(i) = full.col(indices[i]);
  }

  return result;
}

template <typename Scalar, int Rows>
inline Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> select(
    const Eigen::Matrix<Scalar, Rows, Eigen::Dynamic>& full,
    const Eigen::VectorXi& indices)
{
  Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> result(
      full.rows(), indices.size());
  for (int i = 0; i < indices.size(); ++i)
  {
    CHECK_LT(indices[i], full.cols());
    result.col(i) = full.col(indices[i]);
  }

  return result;
}

template <typename Scalar, int Rows>
inline Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> select(
    const Aligned<std::vector, Eigen::Matrix<Scalar, Rows, 1>>& full,
    const Eigen::VectorXi& indices)
{
  static_assert(Rows != Eigen::Dynamic, "Full matrix must have known rows!");
  Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> result(Rows, indices.size());
  for (int i = 0; i < indices.size(); ++i)
  {
    CHECK_GE(indices[i], 0);
    CHECK_LT(indices[i], full.size());
    result.col(i) = full[indices[i]];
  }

  return result;
}

template <typename Scalar, int Rows, typename IndexType>
inline Aligned<std::vector, Eigen::Matrix<Scalar, Rows, 1>> select(
    const Aligned<std::vector, Eigen::Matrix<Scalar, Rows, 1>>& full,
    const std::vector<IndexType>& indices)
    {
  Aligned<std::vector, Eigen::Matrix<Scalar, Rows, 1>> result(indices.size());
  for (size_t i = 0u; i < indices.size(); ++i)
  {
    CHECK_GE(indices[i], 0);
    CHECK_LT(indices[i], full.size());
    result[i] = full[indices[i]];
  }
  return result;
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> selectCols(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& full,
    const Eigen::VectorXi& indices)
{
  CHECK_LT(indices.maxCoeff(), full.cols());
  CHECK_GE(indices.minCoeff(), 0);
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> result(
      full.rows(), indices.size());

  for (int i = 0; i < indices.size(); ++i)
  {
    result.col(i) = full.col(indices[i]);
  }

  return result;
}

}  // namespace rpg_common
namespace rpg = rpg_common;
