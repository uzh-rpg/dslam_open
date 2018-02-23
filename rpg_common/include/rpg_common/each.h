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

#include <Eigen/Dense>

namespace rpg_common {

template <typename ObjectType, typename AllocType>
class VectorEach;
template <typename ScalarType>
class EigenVectorEach;
template <typename ScalarType>
class EigenRowVectorEach;

template <typename ObjectType, typename AllocType>
inline VectorEach<ObjectType, AllocType> each(
    const std::vector<ObjectType, AllocType>& container)
{
  return VectorEach<ObjectType, AllocType>(container);
}
template <typename ScalarType>
inline EigenVectorEach<ScalarType> each(
    const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& matrix)
{
  return EigenVectorEach<ScalarType>(matrix);
}
template <typename ScalarType>
inline EigenRowVectorEach<ScalarType> each(
    const Eigen::Matrix<ScalarType, 1, Eigen::Dynamic>& matrix)
{
  return EigenRowVectorEach<ScalarType>(matrix);
}

template <typename ObjectType, typename AllocType>
class VectorEach {
public:
  explicit inline VectorEach(
      const std::vector<ObjectType, AllocType>& container) :
  container_(container)
  {}

  inline std::vector<bool> operator ==(const ObjectType& reference) const
    {
      std::vector<bool> result;
      for (const ObjectType& object : container_)
      {
        result.push_back(object == reference);
      }
      return result;
    }

  inline std::vector<bool> operator !=(const ObjectType& reference) const
  {
    std::vector<bool> result;
    for (const ObjectType& object : container_)
    {
      result.push_back(object != reference);
    }
    return result;
  }

  template <typename OutType, typename Functor>
  inline std::vector<OutType, AllocType> apply(const Functor& functor) const
  {
    std::vector<OutType, AllocType> result;
    for (const ObjectType& object : container_)
    {
      result.emplace_back(functor(object));
    }
    return result;
  }

private:
  const std::vector<ObjectType, AllocType>& container_;
};

template <typename ScalarType>
class EigenVectorEach {
public:
  explicit inline EigenVectorEach(
      const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& vector) :
      vector_(vector)
  {}

  inline std::vector<bool> operator <(const ScalarType reference) const
  {
    std::vector<bool> result(vector_.size());
    for (int i = 0; i < vector_.size(); ++i)
    {
      result[i] = vector_(i) < reference;
    }
    return result;
  }

  inline std::vector<bool> operator >(const ScalarType reference) const
  {
    std::vector<bool> result(vector_.size());
    for (int i = 0; i < vector_.size(); ++i)
    {
      result[i] = vector_(i) > reference;
    }
    return result;
  }

  inline std::vector<bool> operator !=(const ScalarType reference) const
  {
    std::vector<bool> result;
    for (int i = 0; i < vector_.size(); ++i)
    {
      result.push_back(vector_(i) != reference);
    }
    return result;
  }

private:
  const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& vector_;
};

template <typename ScalarType>
class EigenRowVectorEach {
public:
  explicit inline EigenRowVectorEach(
      const Eigen::Matrix<ScalarType, 1, Eigen::Dynamic>& vector) :
      vector_(vector)
  {}

  inline std::vector<bool> operator <(const ScalarType reference) const
  {
    std::vector<bool> result;
    for (int i = 0; i < vector_.size(); ++i)
    {
      result.push_back(vector_(i) < reference);
    }
    return result;
  }

private:
  const Eigen::Matrix<ScalarType, 1, Eigen::Dynamic>& vector_;
};
}  // namespace rpg_common
namespace rpg = rpg_common;
