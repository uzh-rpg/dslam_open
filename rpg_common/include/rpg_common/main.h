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

#include <gflags/gflags.h>
#include <glog/logging.h>

// Does vanilla initialization of glog and gflags.
// Usage:
// RPG_COMMON_MAIN
// {
//   // same body as usually, with argc and argv
// }
// You will need to add gflags_catkin and glog_catkin
// as dependency to package.xml .

#define RPG_COMMON_MAIN \
int rpg_common_main(int argc, char** argv); \
int main(int argc, char** argv) \
{ \
  google::InitGoogleLogging(argv[0]); \
  google::ParseCommandLineFlags(&argc, &argv, true); \
  google::InstallFailureSignalHandler(); \
  FLAGS_alsologtostderr = true; \
  FLAGS_colorlogtostderr = true; \
  return rpg_common_main(argc, argv); \
} \
int rpg_common_main(int argc, char** argv)
