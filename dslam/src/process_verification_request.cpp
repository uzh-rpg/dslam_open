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

#include "dslam/process_verification_request.h"

#include <glog/logging.h>
#include <rpg_common/aligned.h>
#include <rpg_common/load.h>

#include "dslam/orb.h"
#include "dslam/verify_geometry.h"

namespace dslam {

struct RelocFrame
{
  int robot_i;
  int frame_i;
  rpg::Aligned<std::vector, Eigen::Vector3d> p_C_lm;
  orb::Descs descriptors;
  std::vector<DBoW2::NodeId> word_ids;
};

struct VerificationQuery
{
  const RelocFrame* query;
  const RelocFrame* match;
};

template <typename LineType>
orb::Desc compress(const LineType& double_bytes)
{
  return double_bytes.template cast<unsigned char>();
}

enum class AssociationMode
{
  kOrbDescriptor,
  kWordId
};

void processVerificationRequest(
    const std::string& infile, const std::string& outfile,
    const ORB_SLAM2::ORBVocabulary& vocabulary)
{
  // Load from file.
  AssociationMode association_mode;
  Eigen::MatrixXd serialized;
  rpg::load(infile, &serialized);
  if (serialized.cols() == 38)
  {
    association_mode = AssociationMode::kOrbDescriptor;
  }
  else
  {
    CHECK_EQ(serialized.cols(), 7);
    association_mode = AssociationMode::kWordId;
  }

  auto robot_is = serialized.leftCols<1>();
  auto frame_is = serialized.middleCols<1>(1);
  auto type_is = serialized.middleCols<1>(2);
  Eigen::Matrix<double, Eigen::Dynamic, 32> descriptors;
  Eigen::VectorXd word_ids;
  switch (association_mode)
  {
    case AssociationMode::kOrbDescriptor:
      descriptors = serialized.middleCols<32>(3);
      break;
    case AssociationMode::kWordId:
      word_ids = serialized.col(3);
      break;
  }
  auto p_C_lms = serialized.rightCols<3>().cast<float>();

  // Convert into frames.
  std::vector<dslam::RelocFrame> frames;
  int type_i = -1;
  for (int line_i = 0; line_i < serialized.rows(); ++line_i)
  {
    if (type_is(line_i) != type_i)
    {
      frames.emplace_back();
      type_i = type_is(line_i);
      frames.back().robot_i = robot_is(line_i);
      frames.back().frame_i = frame_is(line_i);
    }
    frames.back().p_C_lm.emplace_back(
        p_C_lms.cast<double>().row(line_i).transpose());
    switch (association_mode)
    {
      case AssociationMode::kOrbDescriptor:
        frames.back().descriptors.emplace_back(
            dslam::compress(descriptors.row(line_i)));
        break;
      case AssociationMode::kWordId:
        frames.back().word_ids.emplace_back(word_ids(line_i));
        break;
    }
  }

  // Convert into queries
  std::vector<dslam::VerificationQuery> queries(frames.size() / 2);
  for (int i = 0; i < frames.size() / 2; ++i)
  {
    queries[i].query = &frames[2 * i];
    queries[i].match = &frames[2 * i + 1];
  }

  std::ofstream out_file(outfile, std::ios_base::out);
  CHECK(out_file.is_open());
  for (const dslam::VerificationQuery& query : queries)
  {
    LOG(INFO) << "Processing query from " << query.query->robot_i << " to " <<
        query.match->robot_i;
    rpg::Sim3 Sim_M_Q;
    bool success;

    switch (association_mode)
    {
      case AssociationMode::kOrbDescriptor:
        success = dslam::verifyGeometry(
            query.match->descriptors, query.query->descriptors,
            query.match->p_C_lm, query.query->p_C_lm, vocabulary, &Sim_M_Q);
        break;
      case AssociationMode::kWordId:
        success = dslam::verifyGeometry(
            query.match->word_ids, query.query->word_ids,
            query.match->p_C_lm, query.query->p_C_lm, vocabulary, &Sim_M_Q);
        break;
    }

    if (success)
    {
      LOG(INFO) << "Got Sim3:\n" << Sim_M_Q;
    }
    else
    {
      LOG(INFO) << "No geometric verification!";
    }
    out_file << query.query->robot_i << " "
        << query.query->frame_i << " "
        << query.match->robot_i << " "
        << query.match->frame_i << " "
        << (success ? 1 : 0) << " " <<
        Eigen::Map<Eigen::Matrix<double, 1, 16>>(
            Sim_M_Q.getTransformationMatrix().data()) << std::endl;
  }
}

}  // namespace dslam
