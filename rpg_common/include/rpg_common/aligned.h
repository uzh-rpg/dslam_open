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

#include <map>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <glog/logging.h>

// See https://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
namespace rpg_common {

// e.g. rpg::Aligned<std::vector, Eigen::Vector2d>
// Consider using Matrix2Xd instead in such cases.
template <template <typename Type, typename Allocator> class StlContainer,
typename EigenType>
using Aligned = StlContainer<
    EigenType, Eigen::aligned_allocator<EigenType>>;

namespace aligned {

template <typename KeyType, typename EigenType>
using Map =
    std::map<KeyType, EigenType, std::less<KeyType>,
    Eigen::aligned_allocator<std::pair<const KeyType, EigenType>>>;

template <typename KeyType, typename EigenType>
using UnorderedMap =
    std::unordered_map<KeyType, EigenType, std::hash<KeyType>,
    std::equal_to<KeyType>,
    Eigen::aligned_allocator<std::pair<const KeyType, EigenType>>>;

template <typename Scalar, int Rows>
Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> toMat(
    const Aligned<std::vector, Eigen::Matrix<Scalar, Rows, 1>>& container)
{
  Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> result(Rows, container.size());
  for (size_t i = 0u; i < container.size(); ++i)
  {
    result.col(i) = container[i];
  }
  return result;
}

}  // namespace aligned
}  // namespace rpg_common
namespace rpg = rpg_common;
