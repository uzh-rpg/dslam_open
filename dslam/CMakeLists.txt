# Copyright (C) 2017-2018 Titus Cieslewski, RPG, University of Zurich, 
#   Switzerland
#   You can contact the author at <titus at ifi dot uzh dot ch>
# Copyright (C) 2017-2018 Siddharth Choudhary, College of Computing,
#   Georgia Institute of Technology, Atlanta, GA, USA
# Copyright (C) 2017-2018 Davide Scaramuzza, RPG, University of Zurich, 
#   Switzerland
#
# This file is part of dslam_open.
#
# dslam_open is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# dslam_open is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with dslam_open. If not, see <http://www.gnu.org/licenses/>.

project(dslam)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-O3)

cs_add_library(${PROJECT_NAME} src/kitti_parser.cpp
                               src/orb_matcher.cpp
                               src/process_verification_request.cpp
                               src/sim_3_solver.cpp
                               src/stata_parser.cpp
                               src/verify_geometry.cpp)

cs_add_executable(dataset_to_vo src/dataset_to_vo.cpp)
target_link_libraries(dataset_to_vo ${PROJECT_NAME})

cs_add_executable(batch_process_verification_request
                  src/batch_process_verification_request.cpp)
target_link_libraries(batch_process_verification_request ${PROJECT_NAME})	

cs_add_executable(verification_request_server
                  src/verification_request_server.cpp)
target_link_libraries(verification_request_server ${PROJECT_NAME})

cs_install()
cs_export()
