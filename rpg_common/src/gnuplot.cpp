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

#include "rpg_common/gnuplot.h"

#include <glog/logging.h>

namespace rpg_common {

Gnuplot::Gnuplot(const bool persist)
{
  pipe_ = popen(persist ? "gnuplot --persist" : "gnuplot", "w");
  // Prevent gnuplot from stealing focus all the time.
  operator <<("set term x11 noraise");
}

void Gnuplot::operator <<(const std::string& string)
{
  fputs((string + std::string("\n")).c_str(), pipe_);
  fflush(pipe_);
}

void Gnuplot::feedAsXy(
    const std::vector<double>& x, const std::vector<double>& y)
{
  CHECK_EQ(x.size(), y.size());
  for (size_t i = 0u; i < x.size(); ++i)
  {
    fprintf(pipe_, "%lf %lf\n", x[i], y[i]);
  }
  fprintf(pipe_, "e\n");
  fflush(pipe_);
}

}  // namespace rpg_common
