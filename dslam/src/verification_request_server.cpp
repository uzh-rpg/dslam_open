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

#include <fstream>
#include <iostream>

#include <ORB_SLAM2/ORBVocabulary.h>
#include <ros/ros.h>
#include <rpg_common/fs.h>
#include <rpg_common/main.h>

#include "dslam/process_verification_request.h"

DECLARE_bool(step_ransac);
DECLARE_bool(debug_optim);
DECLARE_bool(viz_optim);

RPG_COMMON_MAIN
{
  if (FLAGS_step_ransac || FLAGS_debug_optim || FLAGS_viz_optim)
  {
    ros::init(argc, argv, "batch_process_verification_request");
    ros::Time::init();
  }

  if(argc != 4)
  {
    std::cerr << std::endl << "Usage: " << argv[0] <<
        " path_to_verification_request path_to_result_file "
        "path_to_lock_file" << std::endl;
    return 1;
  }
  const std::string request_file(argv[1]);
  const std::string out_file_name(argv[2]);
  const std::string lock_file_name(argv[3]);

  std::shared_ptr<ORB_SLAM2::ORBVocabulary> vocabulary =
      ORB_SLAM2::orb_vocabulary::makeDefault();

  while (true)
  {
    while (!rpg::fs::fileExists(request_file))
    {
      usleep(1000);
    }
    while (rpg::fs::fileExists(lock_file_name))
    {
      usleep(1000);
    }

    std::ofstream lock_file(lock_file_name);
    dslam::processVerificationRequest(
        request_file, out_file_name, *vocabulary);
    std::remove(request_file.c_str());
    lock_file.close();
    std::remove(lock_file_name.c_str());
  }

  return 0;
}
