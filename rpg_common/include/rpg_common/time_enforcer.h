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

#include <stdlib.h>

namespace rpg_common {

// Ensures that time intervals between successive returns from waitUntil()
// have the same length as the difference between the passed virtual times.
class TimeEnforcer
{
public:
  // Returns false if time has already passed.
  bool waitUntil(const int64_t t_virtual_ns);

private:
  bool first_call_happened_ = false;
  int64_t t_virtual_wall_ns_;
};

}  // namespace rpg_common
namespace rpg = rpg_common;
