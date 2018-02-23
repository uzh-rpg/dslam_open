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
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <unordered_set>

#include <Eigen/Dense>
#include <gflags/gflags.h>
#include <ORB_SLAM2/KeyFrame.h>
#include <ORB_SLAM2/MapPoint.h>
#include <ORB_SLAM2/System.h>
#include <rpg_common/aligned.h>
#include <rpg_common/cv_to_eigen.h>
#include <rpg_common/fs.h>
#include <rpg_common/gnuplot.h>
#include <rpg_common/main.h>
#include <rpg_common/stream_utility.h>
#include <rpg_common/time_enforcer.h>

#include "dslam/dataset_parser.h"
#include "dslam/kitti_parser.h"
#include "dslam/stata_parser.h"

DEFINE_string(type, "", "dataset type - kitti or stata");

size_t getWordId(
    const ORB_SLAM2::ORBVocabulary& vocabulary, const cv::Mat& descriptor,
    const size_t word_level)
{
  CHECK_LE(word_level, vocabulary.getDepthLevels());
  const size_t levels_up = vocabulary.getDepthLevels() - word_level;
  DBoW2::WordId word_id;
  DBoW2::NodeId node_id;
  DBoW2::WordValue word_value;
  vocabulary.transform(descriptor, word_id, word_value, &node_id, levels_up);
  return node_id;
}

RPG_COMMON_MAIN
{
  CHECK(!FLAGS_type.empty());

  if(argc != 3)
  {
    std::cerr << std::endl << "Usage: " << argv[0] << " path_to_sequence " <<
        "path_to_orb_settings" << std::endl;
    return 1;
  }
  const std::string path_to_sequence(argv[1]);
  const std::string settings_file_name(argv[2]);

  std::unique_ptr<dslam::DatasetParser> parser;
  Eigen::Vector2i image_dimensions;
  if (FLAGS_type == "kitti")
  {
    parser.reset(new dslam::KittiParser);
    image_dimensions << 1241, 376;
  }
  else if (FLAGS_type == "stata")
  {
    parser.reset(new dslam::StataParser);
    image_dimensions << 640, 480;
  }
  else
  {
    LOG(FATAL) << "Unknown dataset type " << FLAGS_type;
  }

  dslam::DatasetParser::FrameVector frame_vector;
  parser->parse(path_to_sequence, &frame_vector);

  std::shared_ptr<ORB_SLAM2::ORBVocabulary> vocabulary =
      ORB_SLAM2::orb_vocabulary::makeDefault();

  size_t sequence_i = 0u;
  for (size_t frame_i = 0u; frame_i < frame_vector.size(); ++sequence_i)
  {
    // Not using viewer due to singleton implementation.
    constexpr bool kUseViewer = false;
    ORB_SLAM2::System SLAM(
        settings_file_name, ORB_SLAM2::Sensor::STEREO, kUseViewer);
    // Shutting down loop closure, as we want to evaluate for this.
    SLAM.ShutDownLoopClosure();

    std::ofstream pose_out("poses_" + std::to_string(sequence_i) + ".txt");
    std::ofstream time_out("times_" + std::to_string(sequence_i) + ".txt");
    std::ofstream lm_obs_out("lm_obs_" + std::to_string(sequence_i) + ".txt");
    std::ofstream desc_out("descs_" + std::to_string(sequence_i) + ".txt");
    std::ofstream lm_pos_out("lm_pos_" + std::to_string(sequence_i) + ".txt");
    CHECK(pose_out.is_open());
    CHECK(time_out.is_open());
    CHECK(lm_obs_out.is_open());
    CHECK(desc_out.is_open());
    CHECK(lm_pos_out.is_open());
    std::vector<ORB_SLAM2::MapPoint*> point_idx_to_ptr;
    std::unordered_map<ORB_SLAM2::MapPoint*, size_t> point_ptr_to_idx;
    size_t current_frame_i = 0u;

    SLAM.AttachCallbackToNewKeyframe([&](const ORB_SLAM2::KeyFrame& key_frame){
      const Eigen::Matrix4f T_W_C =
          rpg::cvToEigen<float, 4, 4>(key_frame.GetPoseInverse());
      pose_out << Eigen::Map<const Eigen::Matrix<float, 1, 16>>(
          T_W_C.data()) << std::endl;
      time_out << std::fixed << key_frame.mTimeStamp << std::endl;

      const std::vector<ORB_SLAM2::MapPoint*> map_points =
          key_frame.GetMapPointMatches();
      const size_t num_keypoints = map_points.size();
      constexpr size_t kOrbDescriptorSizeBytes = 32u;
      // Descriptors: From row-major CU8 cv to col-major Eigen, through a map.
      Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> descriptors =
          Eigen::Map<
          Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>>(
              key_frame.mDescriptors.data, kOrbDescriptorSizeBytes,
              num_keypoints);

      for (size_t point_i = 0u; point_i < map_points.size(); ++point_i)
      {
        ORB_SLAM2::MapPoint* const map_point = map_points[point_i];
        if (!map_point)
        {
          continue;
        }
        auto found = point_ptr_to_idx.find(map_point);
        if (found == point_ptr_to_idx.end())
        {
          point_idx_to_ptr.emplace_back(map_point);
          found = point_ptr_to_idx.emplace(
              map_point, point_idx_to_ptr.size() - 1u).first;
        }
        lm_obs_out << current_frame_i << " " << found->second << std::endl;
        for (size_t byte_i = 0u; byte_i < kOrbDescriptorSizeBytes; ++byte_i)
        {
          desc_out << static_cast<size_t>(descriptors(byte_i, point_i)) << " ";
        }

        desc_out <<
            getWordId(*vocabulary, key_frame.mDescriptors.row(point_i), 3) <<
            " " <<
            getWordId(*vocabulary, key_frame.mDescriptors.row(point_i), 4) <<
            " ";

        desc_out << std::endl;
      }

      ++current_frame_i;
    });

    rpg::TimeEnforcer time_enforcer;
    bool initializing = true;
    rpg::Aligned<std::vector, Eigen::RowVector2d> pos_2d;
    rpg::Gnuplot plot;
    plot << "set title '2d position'";
    plot << "set size ratio -1";
    for (; frame_i < frame_vector.size(); ++frame_i)
    {
      const dslam::DatasetParser::Frame& frame = frame_vector[frame_i];
      time_enforcer.waitUntil(frame.time_ns);
      CHECK(rpg::fs::fileExists(frame.image_path)) << frame.image_path;
      cv::Mat pose;
      pose = SLAM.TrackStereo(cv::imread(frame.image_left),
                              cv::imread(frame.image_right),
                              frame.time_ns / 1.e9);
      if (pose.empty())
      {
        if (initializing)
        {
          LOG(WARNING) << "Initializing...";
        }
        else
        {
          LOG(WARNING) << "Tracking lost, firing up next instance!";
          break;
        }
      }
      else
      {
        initializing = false;
        const Eigen::Matrix4d T_W_C =
            rpg::cvToEigen<float, 4, 4>(pose).cast<double>().transpose().inverse();
        pos_2d.emplace_back(Eigen::RowVector2d(T_W_C(0, 3), T_W_C(2, 3)));
        plot << "plot '-' w l";
        plot << pos_2d;
      }
    }

    SLAM.Shutdown();

    for (ORB_SLAM2::MapPoint* const map_point : point_idx_to_ptr)
    {
      CHECK_NOTNULL(map_point);
      lm_pos_out << rpg::cvToEigen<float, 3, 1>(
          map_point->GetWorldPos()).transpose() << std::endl;
    }
  }

  return 0;
}
