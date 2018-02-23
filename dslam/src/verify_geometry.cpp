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

#include "dslam/verify_geometry.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <rpg_common/cv_to_eigen.h>
#include <rpg_common/each.h>
#include <rpg_common/eigen_to_cv.h>
#include <rpg_common/pose.h>
#include <rpg_common/select.h>
#include <rpg_common/stream_utility.h>
#include <rpg_common_ros/visualize.h>

#include "dslam/orb_matcher.h"
#include "dslam/sim_3_solver.h"

DEFINE_bool(debug_optim, false,
            "Debug the optimization? (same rviz as step ransac)");
DEFINE_bool(viz_optim, false,
            "Visualize the optimization out? (same rviz as step ransac)");

DEFINE_double(atan_loss, 3.0,
            "Atan loss function scale (none applied if 0.)");
DEFINE_uint64(min_num_inliers, 20,
            "Minimum number of inlier landmarks.");
DEFINE_double(final_inlier_thresh, 1.0,
            "Max distance between final inlier matched points.");

DEFINE_bool(match_by_voc_stats, false,
            "Compare descriptor matches to voc matches?");

namespace dslam {

inline Eigen::Matrix3Xd residual(
    const Eigen::Matrix3Xd& p_M_mpts, const Eigen::Matrix3Xd& p_Q_qpts,
    const Eigen::Matrix3d& R_M_Q, const Eigen::Vector3d& t_M_Q)
{
  const Eigen::Matrix3Xd p_M_qpts = (R_M_Q * p_Q_qpts).colwise() + t_M_Q;
  return p_M_mpts - p_M_qpts;
}

inline Eigen::Matrix3Xd residual(
    const Eigen::Matrix3Xd& p_M_mpts, const Eigen::Matrix3Xd& p_Q_qpts,
    const rpg::Pose& T_M_Q)
{
  return residual(
      p_M_mpts, p_Q_qpts, T_M_Q.getRotation().getRotationMatrix(),
      T_M_Q.getPosition());
}

class PoseRefineCostFunctor
{
public:
  PoseRefineCostFunctor(
      const Eigen::Matrix3Xd& p_M_mpts, const Eigen::Matrix3Xd& p_Q_qpts)
: p_M_mpts_(p_M_mpts), p_Q_qpts_(p_Q_qpts)
{
    CHECK_EQ(p_M_mpts.cols(), p_Q_qpts.cols());
}

  bool operator()(const double* const aaxxyz_M_Q, double* residual) const
  {
    double R_M_Q_data[9];
    ceres::AngleAxisToRotationMatrix(
        aaxxyz_M_Q, ceres::MatrixAdapter<double, 1, 3>(R_M_Q_data));
    Eigen::Map<Eigen::Matrix3d> R_M_Q(R_M_Q_data);
    Eigen::Map<const Eigen::Vector3d> t(aaxxyz_M_Q + 3);
    const Eigen::Matrix3Xd eigen_residual =
        dslam::residual(p_M_mpts_, p_Q_qpts_, R_M_Q, t);
    Eigen::Map<Eigen::Matrix3Xd>(
        residual, 3, eigen_residual.cols()) = eigen_residual;
    return true;
  }

private:
  const Eigen::Matrix3Xd& p_M_mpts_, p_Q_qpts_;
};

rpg::Pose optimizeRelPose(
    const Eigen::Matrix3Xd& p_M_mpts, const Eigen::Matrix3Xd& p_Q_qpts,
    const rpg::Pose& initial_T_M_Q)
{
  ceres::CostFunction* cost_function =
      new ceres::NumericDiffCostFunction<
      PoseRefineCostFunctor, ceres::CENTRAL, ceres::DYNAMIC, 6>(
          new PoseRefineCostFunctor(p_M_mpts, p_Q_qpts),
          ceres::TAKE_OWNERSHIP, 3 * p_M_mpts.cols());
  ceres::Problem problem;
  double aaxxyz[6];
  const Eigen::Matrix4d initial_T_M_Q_mat =
      initial_T_M_Q.getTransformationMatrix();
  ceres::RotationMatrixToAngleAxis(
      ceres::MatrixAdapter<const double, 1, 4>(
          initial_T_M_Q_mat.data()), aaxxyz);
  Eigen::Map<Eigen::Vector3d>(aaxxyz + 3) =
      initial_T_M_Q_mat.topRightCorner<3, 1>();
  const Eigen::Matrix<double, 1, 6> original_aaxxyz =
      Eigen::Map<Eigen::Matrix<double, 1, 6>>(aaxxyz);

  ceres::LossFunction* loss = nullptr;
  if (FLAGS_atan_loss != 0.)
  {
    loss = new ceres::ArctanLoss(FLAGS_atan_loss);
  }
  problem.AddResidualBlock(cost_function, loss, aaxxyz);
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  double R_data[9];
  ceres::AngleAxisToRotationMatrix(
      aaxxyz, ceres::MatrixAdapter<double, 1, 3>(R_data));
  Eigen::Map<Eigen::Matrix3d> R(R_data);
  Eigen::Map<Eigen::Vector3d> t(aaxxyz + 3);
  Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
  T_mat.topLeftCorner<3, 3>() = R;
  T_mat.topRightCorner<3, 1>() = t;
  return rpg::Pose(T_mat);
}

void compareDescMatchToVocMatch(
    const orb::Descs& descs_M, const orb::Descs& descs_Q,
    const ORB_SLAM2::ORBVocabulary& vocabulary, const size_t level,
    const std::vector<int>& orb_matches)
{
  LOG(INFO) <<"With " << pow(10, level) << " words:";

  // Get feature vectors.
  CHECK_EQ(vocabulary.getDepthLevels(), 6u);
  CHECK_LE(level, 6u);
  const size_t levels_up = 6u - level;
  std::vector<cv::Mat> descs_M_cv, descs_Q_cv;
  rpg::eigenToCv(descs_M, &descs_M_cv);
  rpg::eigenToCv(descs_Q, &descs_Q_cv);
  DBoW2::BowVector bow_M, bow_Q;
  DBoW2::FeatureVector feat_M, feat_Q;
  vocabulary.transform(descs_M_cv, bow_M, feat_M, levels_up);
  vocabulary.transform(descs_Q_cv, bow_Q, feat_Q, levels_up);

  // Split matches.
  std::vector<size_t> q_matches =
      rpg::select(rpg::each(orb_matches) != -1);
  std::vector<int> m_matches =
      rpg::select(orb_matches, rpg::each(orb_matches) != -1);

  std::vector<bool> false_negative(q_matches.size(), true);
  size_t unique_true_positives = 0u;
  size_t unique_false_positives = 0u;
  size_t ambiguous_true_positives = 0u;
  feat_Q.forEachCommonNode(feat_M, [&](const DBoW2::NodeId& node_id,
      const std::vector<unsigned int>& q_indices,
      const std::vector<unsigned int>& m_indices){
    // Unique vocab match?
    if (q_indices.size() == 1u && m_indices.size() == 1u)
    {
      for (size_t i = 0u; i < q_matches.size(); ++i)
      {
        if (q_matches[i] == q_indices[0])
        {
          if (m_matches[i] == m_indices[0])
          {
            ++unique_true_positives;
            false_negative[i] = false;
          }
          else
          {
            ++unique_false_positives;
            false_negative[i] = false;
          }
          return;  // "continue" of the lambda.
        }
      }
      // No corresponding match has been found: Another false positive!
      ++unique_false_positives;
      return;
    }
    // Vocab match not unique.
    for (unsigned int q_index : q_indices)
    {
      for (size_t i = 0u; i < q_matches.size(); ++i)
      {
        // Case vocab match corresponds to a descriptor match:
        if (q_matches[i] == q_index && std::count(
            m_indices.begin(), m_indices.end(), m_matches[i]) > 0u)
        {
          ++ambiguous_true_positives;
          false_negative[i] = false;
          break;
        }
      }
    }
  });

  LOG(INFO) << "Unique true positives: " << unique_true_positives;
  LOG(INFO) << "Unique false positives: " << unique_false_positives;
  LOG(INFO) << "Ambiguous true positives: " << ambiguous_true_positives;
  LOG(INFO) << "False negatives:" << rpg::select(false_negative).size();

  if(false)
  {

    LOG(INFO) << feat_Q.size();

    std::vector<size_t> words_Q, words_M;
    for (const DBoW2::FeatureVector::value_type node_indices : feat_Q)
    {
      for (size_t index : node_indices.second)
      {
        if (words_Q.size() < index + 1)
        {
          words_Q.resize(index + 1);
        }
        words_Q[index] = node_indices.first;
      }
    }
    for (const DBoW2::FeatureVector::value_type node_indices : feat_M)
    {
      for (size_t index : node_indices.second)
      {
        if (words_M.size() < index + 1)
        {
          words_M.resize(index + 1);
        }
        words_M[index] = node_indices.first;
      }
    }

    for (size_t i = 0u; i < q_matches.size(); ++i)
    {
      LOG(INFO) << i << " " << q_matches[i] << " " << m_matches[i] <<  "(" <<
          words_Q[q_matches[i]] << ", " << words_M[m_matches[i]] << ")";
      LOG(INFO) << rpg::cvToEigen<unsigned char, 32, 1>(descs_Q_cv[q_matches[i]]).cast<int>().transpose();
      LOG(INFO) << descs_Q[q_matches[i]].cast<int>().transpose();
    }
  }
}

template <typename KeypointDescriptorsType>
std::vector<int> match(const KeypointDescriptorsType& descs_M,
                       const KeypointDescriptorsType& descs_Q,
                       const ORB_SLAM2::ORBVocabulary& vocabulary);

template <>
std::vector<int> match(
    const orb::Descs& descs_M, const orb::Descs& descs_Q,
    const ORB_SLAM2::ORBVocabulary& vocabulary)
{
  ORBMatcher matcher(0.75);
  std::vector<int> orb_matches;
  matcher.searchByBoW(descs_M, descs_Q, &orb_matches);

  if (FLAGS_match_by_voc_stats)
  {
    compareDescMatchToVocMatch(descs_M, descs_Q, vocabulary, 2, orb_matches);
    compareDescMatchToVocMatch(descs_M, descs_Q, vocabulary, 3, orb_matches);
    compareDescMatchToVocMatch(descs_M, descs_Q, vocabulary, 4, orb_matches);
    compareDescMatchToVocMatch(descs_M, descs_Q, vocabulary, 5, orb_matches);
    compareDescMatchToVocMatch(descs_M, descs_Q, vocabulary, 6, orb_matches);
  }

  return orb_matches;

  /*if (FLAGS_match_by_voc)
  {
    matcher.searchByVoc(descs_M, descs_Q, 4, &orb_matches);
  }
  */
}

template <>
std::vector<int> match(
    const std::vector<DBoW2::NodeId>& word_ids_M,
    const std::vector<DBoW2::NodeId>& word_ids_Q,
    const ORB_SLAM2::ORBVocabulary& vocabulary)
{
  ORBMatcher matcher(0.75);
  std::vector<int> orb_matches;
  matcher.searchByVoc(word_ids_M, word_ids_Q, &orb_matches);

  return orb_matches;
}

// Mainly adapted from ORB_SLAM2. For now only single candidate handling.
template <typename KeypointDescriptorsType>
bool verifyGeometry(
    const KeypointDescriptorsType& descs_M,
    const KeypointDescriptorsType& descs_Q,
    const Positions& p_M_mpts, const Positions& p_Q_qpts,
    const ORB_SLAM2::ORBVocabulary& vocabulary, rpg::Sim3* Sim_M_Q)
{
  CHECK_NOTNULL(Sim_M_Q);

  std::vector<int> orb_matches = match(descs_M, descs_Q, vocabulary);

  std::vector<size_t> q_matches =
      rpg::select(rpg::each(orb_matches) != -1);
  std::vector<int> m_matches =
      rpg::select(orb_matches, rpg::each(orb_matches) != -1);

  LOG(INFO) << q_matches.size() << " out of " << orb_matches.size() <<
      " match!";
  if(q_matches.size() < FLAGS_min_num_inliers)
  {
    return false;
  }

  const Positions p_M_mpts_matched = rpg::select(p_M_mpts, m_matches);
  const Positions p_Q_qpts_matched = rpg::select(p_Q_qpts, q_matches);

  // TODO(angle from param, eval)
  Sim3Solver solver(p_M_mpts_matched, p_Q_qpts_matched, 5.);
  solver.setRansacParameters(0.99,20,300);

  std::vector<bool> inliers;
  bool success = solver.find(&inliers);

  if(!success)
  {
    return false;
  }

  // For now skipping:
  // * sim 3 matching
  // * optimization with sim 3 matching
  // * search for more matchable points in neighborhood

  *Sim_M_Q = solver.getSim_A_B();

  const Eigen::Matrix3Xd p_M_mpts_inliers = rpg::aligned::toMat(
      rpg::select(p_M_mpts_matched, inliers)).cast<double>();
  const Eigen::Matrix3Xd p_Q_qpts_inliers = rpg::aligned::toMat(
      rpg::select(p_Q_qpts_matched, inliers)).cast<double>();
  const Eigen::Matrix3Xd p_M_qpts_inliers = Sim_M_Q->operator *(
      Eigen::Matrix3Xd(rpg::aligned::toMat(rpg::select(p_Q_qpts_matched, inliers)).cast<double>()));

  rpg::Pose T = optimizeRelPose(p_M_mpts_inliers, p_Q_qpts_inliers, rpg::Pose(
      Sim_M_Q->getTransformationMatrix()));

  if (FLAGS_atan_loss != 0.)
  {
    const Eigen::RowVectorXd residuals2 =  residual(
        p_M_mpts_inliers, p_Q_qpts_inliers, T).cwiseAbs2().colwise().sum();
    const double tolerance2 = FLAGS_atan_loss * FLAGS_atan_loss;
    std::vector<bool> optim_inliers = rpg::each(residuals2) < tolerance2;

    const Eigen::Matrix3Xd final_p_M_mpts =
        rpg::select(p_M_mpts_inliers, optim_inliers);
    const Eigen::Matrix3Xd final_p_Q_qpts =
        rpg::select(p_Q_qpts_inliers, optim_inliers);
    if (final_p_M_mpts.cols() < FLAGS_min_num_inliers)
    {
      return false;
    }
    T = optimizeRelPose(final_p_M_mpts, final_p_Q_qpts, T);

    const Eigen::RowVectorXd final_residuals2 =  residual(
        p_M_mpts_inliers, p_Q_qpts_inliers, T).cwiseAbs2().colwise().sum();
    const double inner_tolerance2 =
        FLAGS_final_inlier_thresh * FLAGS_final_inlier_thresh;
    std::vector<bool> final_inliers =
        rpg::each(final_residuals2) < inner_tolerance2;
    if (rpg::select(final_inliers).size() < FLAGS_min_num_inliers)
    {
      return false;
    }
  }

  if (FLAGS_debug_optim)
  {
    LOG(INFO) << "Before optim, " << p_M_mpts_inliers.cols() << " points:";
    LOG(INFO) << *Sim_M_Q;
    LOG(INFO) << residual(
        p_M_mpts_inliers, p_Q_qpts_inliers, Sim_M_Q->getTransform())
        .cwiseAbs2().sum();

    const Eigen::Matrix3Xd p_M_qpts = (*Sim_M_Q) * p_Q_qpts_inliers;
    rpg_ros::visualize("p_A_apts", p_M_mpts_inliers);
    rpg_ros::visualize("p_B_bpts", p_Q_qpts_inliers);
    rpg_ros::visualize("p_A_bpts", p_M_qpts);
    rpg_ros::visualize("maches_in_A", p_M_mpts_inliers, p_M_qpts);
    getchar();
  }

  *Sim_M_Q = rpg::Sim3(T, 1);

  if (FLAGS_debug_optim || FLAGS_viz_optim)
  {
    LOG(INFO) << "After optim:";
    LOG(INFO) << *Sim_M_Q;
    LOG(INFO) << residual(
        p_M_mpts_inliers, p_Q_qpts_inliers, Sim_M_Q->getTransform())
        .cwiseAbs2().sum();

    const Eigen::Matrix3Xd p_M_qpts = (*Sim_M_Q) * p_Q_qpts_inliers;
    rpg_ros::visualize("p_A_apts", p_M_mpts_inliers);
    rpg_ros::visualize("p_B_bpts", p_Q_qpts_inliers);
    rpg_ros::visualize("p_A_bpts", p_M_qpts);
    rpg_ros::visualize("maches_in_A", p_M_mpts_inliers, p_M_qpts);
    if (FLAGS_debug_optim)
    {
      getchar();
    }
  }

  return true;
}

template bool verifyGeometry(
    const orb::Descs& descs_M, const orb::Descs& descs_Q,
    const Positions& p_M_mpts, const Positions& p_Q_qpts,
    const ORB_SLAM2::ORBVocabulary& vocabulary, rpg::Sim3* Sim_M_Q);

template bool verifyGeometry(
    const std::vector<DBoW2::NodeId>& descs_M,
    const std::vector<DBoW2::NodeId>& descs_Q,
    const Positions& p_M_mpts, const Positions& p_Q_qpts,
    const ORB_SLAM2::ORBVocabulary& vocabulary, rpg::Sim3* Sim_M_Q);

}  // namespace dslam
