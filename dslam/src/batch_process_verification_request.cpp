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

#include <iostream>

#include <ros/ros.h>
#include <rpg_common/main.h>

#include "dslam/process_verification_request.h"

DECLARE_bool(step_ransac);

RPG_COMMON_MAIN
{
  if (FLAGS_step_ransac)
  {
    ros::init(argc, argv, "batch_process_verification_request");
    ros::Time::init();
  }

  if(argc != 3)
  {
    std::cerr << std::endl << "Usage: " << argv[0] <<
        " path_to_verification_request path_to_result_file" << std::endl;
    return 1;
  }
  const std::string request_file(argv[1]);
  const std::string out_file_name(argv[2]);

  std::shared_ptr<ORB_SLAM2::ORBVocabulary> vocabulary =
        ORB_SLAM2::orb_vocabulary::makeDefault();

  dslam::processVerificationRequest(request_file, out_file_name, *vocabulary);

  return 0;
}
