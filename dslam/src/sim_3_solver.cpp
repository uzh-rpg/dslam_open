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

#include "dslam/sim_3_solver.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ORB_SLAM2/Sim3Solver.h>
#include <rpg_common/cv_to_eigen.h>
#include <rpg_common/each.h>
#include <rpg_common/eigen_to_cv.h>
#include <rpg_common/pose.h>
#include <rpg_common_ros/visualize.h>

DEFINE_bool(step_ransac, false, "Step through ransac for debugging?");

namespace dslam {

Sim3Solver::Sim3Solver(
    const Positions& p_A_apts, const Positions& p_B_bpts,
    const double inlier_threshold_deg)
: p_A_apts_(p_A_apts), p_B_bpts_(p_B_bpts), N_(p_A_apts.size()),
  inlier_threshold_cos_(cos(inlier_threshold_deg * M_PI / 180)),
  best_num_inliers_(0u)
{
  CHECK_EQ(N_, p_B_bpts_.size());
  if (FLAGS_step_ransac)
  {
    rpg_ros::visualize("p_A_apts", p_A_apts);
    rpg_ros::visualize("p_B_bpts", p_B_bpts);
    rpg_ros::visualize("matches", p_A_apts, p_B_bpts);
  }
}

void Sim3Solver::setRansacParameters(
    const double probability, const int min_inliers,
    const int max_iterations)
{
  min_inliers_ = min_inliers;
  const float epsilon = static_cast<float>(min_inliers_) / N_;
  const int n_iterations =
      ceil(log(1 - probability) / log(1 - pow(epsilon, 3)));
  max_iterations_ = std::max(1, std::min(n_iterations, max_iterations));
}

bool Sim3Solver::find(std::vector<bool>* inliers)
{
  CHECK_NOTNULL(inliers)->resize(N_, false);
  if (N_ < min_inliers_)
  {
    return false;
  }

  for (size_t i = 0u; i < max_iterations_; ++i)
  {
    std::vector<size_t> available_indices(N_);
    std::iota(available_indices.begin(), available_indices.end(), 0u);

    Positions model_base_a;
    Positions model_base_b;

    for (int i = 0; i < 3; ++i)
    {
      const int indexindex = rand() % available_indices.size();
      const size_t index = available_indices[indexindex];
      model_base_a.emplace_back(p_A_apts_[index]);
      model_base_b.emplace_back(p_B_bpts_[index]);
      available_indices[indexindex] = available_indices.back();
      available_indices.pop_back();
    }

    rpg::Sim3 Sim_A_B;
    if (!minimalEstimate(model_base_a, model_base_b, &Sim_A_B))
    {
      LOG(WARNING) << "Min estimate failed!";
      continue;
    }
    handleModel(Sim_A_B);
  }

  LOG(INFO) << "At most " << best_num_inliers_ << " inliers!";

  *inliers = best_inliers_;
  return best_num_inliers_ > min_inliers_;
}

rpg::Sim3 Sim3Solver::getSim_A_B() const
{
  return best_Sim_A_B_;
}

bool Sim3Solver::minimalEstimate(
      const Positions& p_A_apts, const Positions& p_B_bpts,
      rpg::Sim3* Sim_A_B) const
{
  CHECK_EQ(p_A_apts.size(), 3u);
  CHECK_EQ(p_B_bpts.size(), 3u);
  CHECK_NOTNULL(Sim_A_B);

  for (size_t i = 0u; i < 3u; ++i)
  {
    CHECK(!p_A_apts[i].hasNaN());
    CHECK(!p_B_bpts[i].hasNaN());
    CHECK_GT(p_A_apts[i].cwiseAbs().minCoeff(), 0.);
    CHECK_GT(p_B_bpts[i].cwiseAbs().minCoeff(), 0.);
  }

  Eigen::Matrix3d R_A_B;
  Eigen::Vector3d p_A_B;
  float scale_A_B;

  cv::Mat p_A_apts_cv, p_B_bpts_cv;
  if (p_A_apts == p_B_bpts)
  {
    // For some reason, this does not work with ComputeSim3!
    // Not realistic anyways, but can happen with the faked overlap.
    R_A_B = Eigen::Matrix3d::Identity();
    p_A_B = Eigen::Vector3d::Zero();
    scale_A_B = 1.;
  }
  else
  {
    rpg::eigenToCv(p_A_apts, &p_A_apts_cv);
    rpg::eigenToCv(p_B_bpts, &p_B_bpts_cv);

    cv::Mat R_A_B_cv, p_A_B_cv;


    p_A_apts_cv.convertTo(p_A_apts_cv, CV_32F);
    p_B_bpts_cv.convertTo(p_B_bpts_cv, CV_32F);
    ORB_SLAM2::Sim3Solver::ComputeSim3(
        p_A_apts_cv, p_B_bpts_cv, true,
        &R_A_B_cv, &scale_A_B, &p_A_B_cv);

    R_A_B = rpg::cvToEigen<float, 3, 3>(R_A_B_cv).cast<double>();
    p_A_B = rpg::cvToEigen<float, 3, 1>(p_A_B_cv).cast<double>();
  }

  if (std::isnan(scale_A_B) || scale_A_B < 1e-5)
  {
    LOG(WARNING) << "Bad scale!" << scale_A_B;
    return false;
  }
  if (R_A_B.hasNaN())
  {
    for (size_t i = 0u; i < 3u; ++i)
    {
      LOG(INFO) << p_A_apts[i].transpose();
    }
    LOG(INFO) << p_A_apts_cv;
    for (size_t i = 0u; i < 3u; ++i)
    {
      LOG(INFO) << p_B_bpts[i].transpose();
    }
    LOG(INFO) << p_B_bpts_cv;
    LOG(WARNING) << "NaN rotation!" << R_A_B;
    return false;
  }
  if (p_A_B.hasNaN())
  {
    LOG(WARNING) << "NaN position!" << p_A_B;
    return false;
  }

  *Sim_A_B = rpg::Sim3(rpg::Pose(
      rpg::Pose::Rotation::fromApproximateRotationMatrix(R_A_B), p_A_B),
                   scale_A_B);
  return true;
}

void Sim3Solver::handleModel(const rpg::Sim3& Sim_A_B_model)
{
  const Positions p_A_bpts = rpg::each(p_B_bpts_).apply<Eigen::Vector3d>(
      [&](const Eigen::Vector3d& p_B_bpt){
    return Sim_A_B_model * p_B_bpt;
  });
  const Positions p_B_apts = rpg::each(p_A_apts_).apply<Eigen::Vector3d>(
      [&](const Eigen::Vector3d& p_A_apt){
    return Sim_A_B_model.inverse() * p_A_apt;
  });

  Eigen::RowVectorXd cosines_in_A(N_);
  Eigen::RowVectorXd cosines_in_B(N_);
  std::vector<bool> inliers(N_);
  for (size_t i = 0u; i < N_; ++i)
  {
    cosines_in_A[i] = p_A_bpts[i].dot(p_A_apts_[i]) /
        p_A_bpts[i].norm() / p_A_apts_[i].norm();
    cosines_in_B[i] = p_B_apts[i].dot(p_B_bpts_[i]) /
        p_B_apts[i].norm() / p_B_bpts_[i].norm();

    // Cosine must be GREATER than cosine threshold (= smaller angle).
    inliers[i] = cosines_in_A[i] > inlier_threshold_cos_ &&
        cosines_in_B[i] > inlier_threshold_cos_;
  }

  const size_t num_inliers = std::count_if(
      inliers.begin(), inliers.end(), [](const bool x){ return x;});

  if (FLAGS_step_ransac)
  {
    rpg_ros::visualize("p_A_bpts", p_A_bpts);
    rpg_ros::visualize("maches_in_A", p_A_apts_, p_A_bpts);
    LOG(INFO) << cosines_in_A.array().acos() * 180 / M_PI;
    LOG(INFO) << cosines_in_B.array().acos() * 180 / M_PI;
    LOG(INFO) << std::acos(inlier_threshold_cos_) * 180 / M_PI;
    LOG(INFO) << num_inliers;
    getchar();
  }

  if (num_inliers > best_num_inliers_)
  {
    best_inliers_ = inliers;
    best_Sim_A_B_ = Sim_A_B_model;
    best_num_inliers_ = num_inliers;
  }
}

}  // namespace dslam
