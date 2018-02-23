// Copyright (C) 2017-2018 Titus Cieslewski, RPG, University of Zurich, 
//   Switzerland
//   You can contact the author at <titus at ifi dot uzh dot ch>
// Copyright (C) 2017-2018 Siddharth Choudhary, College of Computing,
//   Georgia Institute of Technology, Atlanta, GA, USA
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

#include <ORB_SLAM2/ORBVocabulary.h>

#include "dslam/orb.h"
#include "dslam/positions.h"

namespace dslam {

class ORBMatcher
{
public:
  // No orientation check for now.
  explicit ORBMatcher(const float nn_ratio);

  // matches: Indices of M matched to i-th descriptor of Q. -1 if unmatched.
  void searchByBoW(
      const orb::Descs& descs_M, const orb::Descs& descs_Q,
      std::vector<int>* matches) const;
  void searchByVoc(
      const std::vector<DBoW2::NodeId>& word_ids_M,
      const std::vector<DBoW2::NodeId>& word_ids_Q,
      std::vector<int>* matches) const;

private:
  float nn_ratio_;
  static std::shared_ptr<ORB_SLAM2::ORBVocabulary> vocabulary_;
};

}  // namespace dslam
