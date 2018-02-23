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

#include "dslam/kitti_parser.h"

#include <glog/logging.h>
#include <rpg_common/fs.h>
#include <rpg_common/load.h>

namespace dslam
{

void KittiParser::parse(
    const std::string& dataset_path, FrameVector* result) const
{
  CHECK_NOTNULL(result)->clear();
  CHECK(rpg::fs::pathExists(dataset_path));
  const std::string timestamp_file = dataset_path + "/times.txt";
  CHECK(rpg::fs::fileExists(timestamp_file));

  Eigen::VectorXd times_s;
  rpg::load(timestamp_file, &times_s);
  result->resize(times_s.rows());

  for (int i = 0; i < times_s.rows(); ++i)
  {
    char image_name[7];
    sprintf(image_name, "%06d", i);

    (*result)[i].time_ns = times_s(i) * 1e9;
    (*result)[i].image_path = dataset_path + "/image_0/" + image_name + ".png";
    (*result)[i].image_left = dataset_path + "/image_0/" + image_name + ".png";
    (*result)[i].image_right = dataset_path + "/image_1/" + image_name + ".png";
  }
}

} // namespace dslam
