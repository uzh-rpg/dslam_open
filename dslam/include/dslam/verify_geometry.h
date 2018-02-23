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
#include <rpg_common/sim3.h>

#include "dslam/orb.h"
#include "dslam/positions.h"

namespace dslam {

// M: matchee, Q: query
template <typename KeypointDescriptorsType>
bool verifyGeometry(
    const KeypointDescriptorsType& descs_M,
    const KeypointDescriptorsType& descs_Q,
    const Positions& p_M_mpts, const Positions& p_Q_qpts,
    const ORB_SLAM2::ORBVocabulary& vocabulary, rpg::Sim3* Sim_M_Q);

}  // namespace dslam
