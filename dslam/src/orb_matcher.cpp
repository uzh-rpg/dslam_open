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

#include "dslam/orb_matcher.h"

#include <ORB_SLAM2/ORBmatcher.h>
#include <rpg_common/eigen_to_cv.h>
#include <rpg_common/hamming_distance.h>

namespace dslam {

std::shared_ptr<ORB_SLAM2::ORBVocabulary> ORBMatcher::vocabulary_;

ORBMatcher::ORBMatcher(const float nn_ratio)
: nn_ratio_(nn_ratio)
{
  if (!vocabulary_)
  {
    vocabulary_ = ORB_SLAM2::orb_vocabulary::makeDefault();
  }
}

void ORBMatcher::searchByBoW(
    const orb::Descs& descs_M, const orb::Descs& descs_Q,
    std::vector<int>* matches) const
{
  // Relation to ORB_SLAM imp: pKF is M, F is Q
  CHECK_NOTNULL(matches)->resize(descs_Q.size(), -1);

  std::vector<cv::Mat> descs_M_cv, descs_Q_cv;
  rpg::eigenToCv(descs_M, &descs_M_cv);
  rpg::eigenToCv(descs_Q, &descs_Q_cv);

  constexpr int kLevelsUp = 6;
  DBoW2::BowVector bow_M, bow_Q;
  DBoW2::FeatureVector feat_M, feat_Q;
  vocabulary_->transform(descs_M_cv, bow_M, feat_M, kLevelsUp);
  vocabulary_->transform(descs_Q_cv, bow_Q, feat_Q, kLevelsUp);

  if (VLOG_IS_ON(3))
  {
    for (const DBoW2::FeatureVector::value_type& node_indices : feat_M)
    {
      std::cout << node_indices.first << ":" << node_indices.second.size() << " ";
    }
    std::cout << std::endl;
    for (const DBoW2::FeatureVector::value_type& node_indices : feat_Q)
    {
      std::cout << node_indices.first << ":" << node_indices.second.size() << " ";
    }
    std::cout << std::endl;
  }

  feat_Q.forEachCommonNode(feat_M, [&](
      const DBoW2::NodeId& node_id,
      const std::vector<unsigned int>& q_indices,
      const std::vector<unsigned int>& m_indices)
      {
    for (const size_t m_index : m_indices)
    {
      const orb::Desc& desc_M = descs_M[m_index];
      CHECK_EQ(desc_M.rows(), orb::kByteSize);
      int best_dist_1 = 256;
      int best_dist_2 = 256;
      int best_q = -1;
      for (const size_t q_index : q_indices)
      {
        if ((*matches)[q_index] != -1)
        {
          continue;
        }

        const orb::Desc& desc_Q = descs_Q[q_index];
        CHECK_EQ(desc_Q.rows(), orb::kByteSize);
        const int dist = rpg::hammingDistance(desc_M, desc_Q);
        if (dist < best_dist_1)
        {
          best_dist_2 = best_dist_1;
          best_dist_1 = dist;
          best_q = q_index;
        }
        else if (dist < best_dist_2)
        {
          best_dist_2 = dist;
        }
      }

      VLOG(200) << "best dists are " << best_dist_1 << " " << best_dist_2;

      if (best_dist_1 < ORB_SLAM2::ORBmatcher::TH_LOW)
      {
        if (best_dist_1 < nn_ratio_ * best_dist_2)
        {
          (*matches)[best_q] = m_index;
        }
      }
    }
      });
}

void ORBMatcher::searchByVoc(
    const std::vector<DBoW2::NodeId>& word_ids_M,
    const std::vector<DBoW2::NodeId>& word_ids_Q,
    std::vector<int>* matches) const
{
  CHECK_NOTNULL(matches)->resize(word_ids_Q.size(), -1);

  DBoW2::FeatureVector feat_M, feat_Q;
  for (size_t i = 0u; i < word_ids_M.size(); ++i)
  {
    feat_M.addFeature(word_ids_M[i], i);
  }
  for (size_t i = 0u; i < word_ids_Q.size(); ++i)
  {
    feat_Q.addFeature(word_ids_Q[i], i);
  }

  feat_Q.forEachCommonNode(feat_M, [&](const DBoW2::NodeId& node_id,
      const std::vector<unsigned int>& q_indices,
      const std::vector<unsigned int>& m_indices){
    // Unique vocab match?
    if (q_indices.size() == 1u && m_indices.size() == 1u)
    {
      (*matches)[q_indices[0]] = m_indices[0];
    }
  });

}

}  // namespace dslam
