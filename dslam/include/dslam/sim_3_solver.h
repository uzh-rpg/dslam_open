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

#include <rpg_common/sim3.h>

#include "dslam/positions.h"

namespace dslam {

class Sim3Solver
{
public:
  // For now: Inliers are considered based on error between bearing vectors in
  // degrees. Keeps interfaces a bit simple.
  Sim3Solver(
      const Positions& p_A_apts, const Positions& p_B_bpts,
      const double inlier_threshold_deg);

  void setRansacParameters(
      const double probability, const int min_inliers,
      const int max_iterations);

  bool find(std::vector<bool>* inliers);

  rpg::Sim3 getSim_A_B() const;

private:
  // Estimate from 3 correspondences.
  bool minimalEstimate(
      const Positions& p_A_apts, const Positions& p_B_bpts,
      rpg::Sim3* Sim_A_B) const;

  // Check inliers and replace best if applicable.
  void handleModel(const rpg::Sim3& Sim_A_B_model);

  // Settings
  const Positions& p_A_apts_;
  const Positions& p_B_bpts_;
  const size_t N_;
  const double inlier_threshold_cos_;
  int min_inliers_ = 0;
  int max_iterations_ = 0;

  // State
  rpg::Sim3 best_Sim_A_B_;
  size_t best_num_inliers_;
  std::vector<bool> best_inliers_;
};

}  // namespace dslam
