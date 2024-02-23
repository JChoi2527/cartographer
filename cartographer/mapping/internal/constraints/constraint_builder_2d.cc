/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

bool MatchSubmap::s_full_match_submap_ = false; // static init
bool MatchSubmap::s_is_first_match_done = false; // static init
bool MatchSubmap::s_local_match_submap_ = false; // static init

namespace cartographer {
namespace mapping {
namespace constraints {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();
static auto* kNumSubmapScanMatchersMetric = metrics::Gauge::Null();

transform::Rigid2d ComputeSubmapPose(const Submap2D& submap) {
  return transform::Project2D(submap.local_pose());
}

ConstraintBuilder2D::ConstraintBuilder2D(
    const constraints::proto::ConstraintBuilderOptions& options,
    common::ThreadPoolInterface* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      finish_node_task_(absl::make_unique<common::Task>()),
      when_done_task_(absl::make_unique<common::Task>()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) 
{
  this->constraintBuilderMinScore = options_.min_score();
  this->constraintBuilderMinScoreDefault = options_.min_score();

}

ConstraintBuilder2D::~ConstraintBuilder2D() {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder2D::MaybeAddConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) {
    return;
  }
  if (!per_submap_sampler_
           .emplace(std::piecewise_construct, std::forward_as_tuple(submap_id),
                    std::forward_as_tuple(options_.sampling_ratio()))
           .first->second.Pulse()) {
    return;
  }

  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, false, /* match_full_submap */
                      constant_data, initial_relative_pose, *scan_matcher,
                      constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}


void ConstraintBuilder2D::MaybeAddConstraint_modi(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const transform::Rigid2d& globalPose2dOfNode,
    const transform::Rigid2d& localToGlobalTfSubmap)
{
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) {
    return;
  }

  //if node global pose is very different from submap_based_global_pose => cancel;
  //by jylee
  const transform::Rigid2d node_global_pose_submap_origin =
      localToGlobalTfSubmap * ComputeSubmapPose(*submap) * initial_relative_pose;
  if ((globalPose2dOfNode.inverse()*node_global_pose_submap_origin).
      translation().norm()> 1.0  // 1.3 means meter..should be parametered.
      || std::abs(
          (globalPose2dOfNode.inverse()*node_global_pose_submap_origin).normalized_angle()) > 0.314)


  {
    //std::cout << "reject by global difference!!  " << node_id << "  " << submap_id << std::endl;
    //std::cout << "differ : " << localToGlobalTfSubmap.translation().norm() << std::endl;
    return;
  }


  if (!per_submap_sampler_
           .emplace(std::piecewise_construct, std::forward_as_tuple(submap_id),
                    std::forward_as_tuple(options_.sampling_ratio()))
           .first->second.Pulse()) {
    return;
  }



  //prevent from comparing newly made submap in traj 1  with traj0 nodes.
  if (submap_id.trajectory_id == 1 && node_id.trajectory_id==0) return;

  //std::cout << "accepted by global difference!!  " << node_id << "  " << submap_id << std::endl;
  //std::cout << "differ : " << localToGlobalTfSubmap.translation().norm() << std::endl;


  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint_LocalCase(submap_id, submap, node_id,
                      constant_data, initial_relative_pose, *scan_matcher,
                      constraint,
                      globalPose2dOfNode,localToGlobalTfSubmap);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, true, /* match_full_submap */
                      constant_data, transform::Rigid2d::Identity(),
                      *scan_matcher, constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::MaybeAddGlobalConstraint_modi(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& globalPose2dOfNode,
    const transform::Rigid2d& localToGlobalTfSubmap) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint_GlobalCase(submap_id, submap, node_id,  /* match_full_submap */
                      constant_data, transform::Rigid2d::Identity(),
                      *scan_matcher, constraint,
                      globalPose2dOfNode, localToGlobalTfSubmap);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::NotifyEndOfNode() {
  absl::MutexLock locker(&mutex_);
  CHECK(finish_node_task_ != nullptr);
  finish_node_task_->SetWorkItem([this] {
    absl::MutexLock locker(&mutex_);
    ++num_finished_nodes_;
  });
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));
  finish_node_task_ = absl::make_unique<common::Task>();
  when_done_task_->AddDependency(finish_node_task_handle);
  ++num_started_nodes_;
}

void ConstraintBuilder2D::WhenDone(
    const std::function<void(const ConstraintBuilder2D::Result&)>& callback) {
  absl::MutexLock locker(&mutex_);
  CHECK(when_done_ == nullptr);
  // TODO(gaschler): Consider using just std::function, it can also be empty.
  when_done_ = absl::make_unique<std::function<void(const Result&)>>(callback);
  CHECK(when_done_task_ != nullptr);
  when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
  thread_pool_->Schedule(std::move(when_done_task_));
  when_done_task_ = absl::make_unique<common::Task>();
}

const ConstraintBuilder2D::SubmapScanMatcher*
ConstraintBuilder2D::DispatchScanMatcherConstruction(const SubmapId& submap_id,
                                                     const Grid2D* const grid) {
  CHECK(grid);
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return &submap_scan_matchers_.at(submap_id);
  }
  auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
  submap_scan_matcher.grid = grid;
  auto& scan_matcher_options = options_.fast_correlative_scan_matcher_options();
  auto scan_matcher_task = absl::make_unique<common::Task>();
  scan_matcher_task->SetWorkItem(
      [&submap_scan_matcher, &scan_matcher_options]() {
        submap_scan_matcher.fast_correlative_scan_matcher =
            absl::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(
                *submap_scan_matcher.grid, scan_matcher_options);
      });
  submap_scan_matcher.creation_task_handle =
      thread_pool_->Schedule(std::move(scan_matcher_task));
  return &submap_scan_matchers_.at(submap_id);
}

void ConstraintBuilder2D::ComputeConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, bool match_full_submap,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const SubmapScanMatcher& submap_scan_matcher,
    std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint) {
  CHECK(submap_scan_matcher.fast_correlative_scan_matcher);
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;

  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'filtered_gravity_aligned_point_cloud' in node j,
  // - the initial guess 'initial_pose' for (map <- node j),
  // - the result 'pose_estimate' of Match() (map <- node j).
  // - the ComputeSubmapPose() (map <- submap i)
  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  if (match_full_submap) {
    kGlobalConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
            constant_data->filtered_gravity_aligned_point_cloud,
            options_.global_localization_min_score(), &score, &pose_estimate)) {
      CHECK_GT(score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      kGlobalConstraintsFoundMetric->Increment();
      kGlobalConstraintScoresMetric->Observe(score);
      MatchSubmap::setFullMatchSubmap(true); // added by Gunther
    } else {
      return;
    }
  } else {
    kConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->Match(
            initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
            constraintBuilderMinScore, &score, &pose_estimate)) {
      // We've reported a successful local match.
      //CHECK_GT(score, constraintBuilderMinScore);
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(score);
      MatchSubmap::setLocalMatchSubmap(true);
    } else {
      return;
    }
  }
  {
    absl::MutexLock locker(&mutex_);
    score_histogram_.Add(score);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher.grid, &pose_estimate,
                            &unused_summary);

  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;
  if (firstTime)
  {
    constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    10000.0,
                                    110000.0},
                                   Constraint::INTER_SUBMAP});
    firstTime= false;
  }
  else if (match_full_submap)
  {
    constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight()*0.01,
                                    options_.loop_closure_rotation_weight()*0.01},
                                   Constraint::INTER_SUBMAP});
  } else
  {
    constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight(),
                                    options_.loop_closure_rotation_weight()},
                                   Constraint::INTER_SUBMAP});
  }
  
  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalized_angle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}


void ConstraintBuilder2D::ComputeConstraint_LocalCase(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const SubmapScanMatcher& submap_scan_matcher,
    std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint,
    const transform::Rigid2d& globalPose2dOfNode,
    const transform::Rigid2d& localToGlobalTfSubmap ) {
  CHECK(submap_scan_matcher.fast_correlative_scan_matcher);
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;

        // The 'constraint_transform' (submap i <- node j) is computed from:
        // - a 'filtered_gravity_aligned_point_cloud' in node j,
        // - the initial guess 'initial_pose' for (map <- node j),
        // - the result 'pose_estimate' of Match() (map <- node j).
        // - the ComputeSubmapPose() (map <- submap i)
  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

        // Compute 'pose_estimate' in three stages:
        // 1. Fast estimate using the fast correlative scan matcher.
        // 2. Prune if the score is too low.
        // 3. Refine.

  kConstraintsSearchedMetric->Increment();
  if (submap_scan_matcher.fast_correlative_scan_matcher->Match(
          initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
          constraintBuilderMinScore, &score, &pose_estimate)) {
    // We've reported a successful local match.
    //CHECK_GT(score, constraintBuilderMinScore);
    kConstraintsFoundMetric->Increment();
    kConstraintScoresMetric->Observe(score);
    MatchSubmap::setLocalMatchSubmap(true);
  } else {
    return;
  }

  {
    absl::MutexLock locker(&mutex_);
    score_histogram_.Add(score);
  }

        // Use the CSM estimate as both the initial and previous pose. This has the
        // effect that, in the absence of better information, we prefer the original
        // CSM estimate.
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher.grid, &pose_estimate,
                            &unused_summary);

  //if node global pose is very different from submap_based_global_pose => cancel;
  //by jylee
  const transform::Rigid2d node_global_pose_submap_origin =
      localToGlobalTfSubmap * pose_estimate;
  if ((globalPose2dOfNode.inverse()*node_global_pose_submap_origin).
      translation().norm()> 1.3  // 1.3 means meter..should be parametered.
      || std::abs(
          (globalPose2dOfNode.inverse()*node_global_pose_submap_origin).normalized_angle()) > 0.314)


  {
    //std::cout << "reject by global difference!!  " << node_id << "  " << submap_id << std::endl;
    //std::cout << "differ : " << localToGlobalTfSubmap.translation().norm() << std::endl;
    return;
  }


  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;

    constraint->reset(new Constraint{submap_id,
                                     node_id,
                                     {transform::Embed3D(constraint_transform),
                                      options_.loop_closure_translation_weight(),
                                      options_.loop_closure_rotation_weight()},
                                     Constraint::INTER_SUBMAP});

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;

    const transform::Rigid2d difference =
        initial_pose.inverse() * pose_estimate;
    info << " differs by translation " << std::setprecision(2)
         << difference.translation().norm() << " rotation "
         << std::setprecision(3) << std::abs(difference.normalized_angle());

    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}






void ConstraintBuilder2D::ComputeConstraint_GlobalCase(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const SubmapScanMatcher& submap_scan_matcher,
    std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint,
    const transform::Rigid2d& globalPose2dOfNode,
    const transform::Rigid2d& localToGlobalTfSubmap) {
  CHECK(submap_scan_matcher.fast_correlative_scan_matcher);
  const transform::Rigid2d submapLocalPose = ComputeSubmapPose(*submap);
  const transform::Rigid2d submapOriginGlobalPose = localToGlobalTfSubmap * submapLocalPose;
  const transform::Rigid2d initial_pose = submapLocalPose * initial_relative_pose;

  //if ( (submapOriginGlobalPose.inverse()*globalPose2dOfNode).translation().norm() >
  //    options_.max_constraint_distance()) {
  //  return;
  //}

  //prevent from comparing newly made submap in traj 1  with traj0 nodes.
  if (submap_id.trajectory_id == 1 && node_id.trajectory_id == 0) return;



        // The 'constraint_transform' (submap i <- node j) is computed from:
        // - a 'filtered_gravity_aligned_point_cloud' in node j,
        // - the initial guess 'initial_pose' for (map <- node j),
        // - the result 'pose_estimate' of Match() (map <- node j).
        // - the ComputeSubmapPose() (map <- submap i)
  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

        // Compute 'pose_estimate' in three stages:
        // 1. Fast estimate using the fast correlative scan matcher.
        // 2. Prune if the score is too low.
        // 3. Refine.
  //always match_full_submap case
  kGlobalConstraintsSearchedMetric->Increment();
  if (submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
          constant_data->filtered_gravity_aligned_point_cloud,
          options_.global_localization_min_score(), &score, &pose_estimate)) {
    CHECK_GT(score, options_.global_localization_min_score());
    CHECK_GE(node_id.trajectory_id, 0);
    CHECK_GE(submap_id.trajectory_id, 0);

    kGlobalConstraintsFoundMetric->Increment();
    kGlobalConstraintScoresMetric->Observe(score);
    MatchSubmap::setFullMatchSubmap(true); // added by Gunther

    if (!firstTime)
    {
      const transform::Rigid2d node_global_pose_submap_origin =
          localToGlobalTfSubmap * pose_estimate;
      if ((globalPose2dOfNode.inverse()*node_global_pose_submap_origin).
          translation().norm()> 2.0   // 1.3 means meter..should be parametered.
          || std::abs(
          (globalPose2dOfNode.inverse()*node_global_pose_submap_origin).normalized_angle()) >0.524)

      {
        std::cout << "reject by fullmatch(fastcorrel) difference!!  " << node_id << "  " << submap_id << std::endl;
        std::cout << "differ : "
                  << (globalPose2dOfNode.inverse()*node_global_pose_submap_origin).
                    translation().norm() << "  " 
                  << (globalPose2dOfNode.inverse()*node_global_pose_submap_origin).normalized_angle()  
                  << std::endl;
        return;
      }
      else
      {
        std::cout << "accept by fullmatch(fastcorrel) difference!!  " << node_id << "  " << submap_id << std::endl;
        std::cout << "differ : "
                  << (globalPose2dOfNode.inverse()*node_global_pose_submap_origin).
                    translation().norm()
                  << std::endl;
      }
    }

  } else {
    return;
  }


  {
    absl::MutexLock locker(&mutex_);
    score_histogram_.Add(score);
  }

        // Use the CSM estimate as both the initial and previous pose. This has the
        // effect that, in the absence of better information, we prefer the original
        // CSM estimate.
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher.grid, &pose_estimate,
                            &unused_summary);

  //if node global pose is very different from submap_based_global_pose => cancel;
  //by jylee
  if (!firstTime)
  {
    const transform::Rigid2d node_global_pose_submap_origin =
        localToGlobalTfSubmap * pose_estimate;
    if ((globalPose2dOfNode.inverse()*node_global_pose_submap_origin).
        translation().norm()> 2.0   // 1.3 means meter..should be parametered.
        || std::abs(
        (globalPose2dOfNode.inverse()*node_global_pose_submap_origin).normalized_angle()) >0.524)

    {
      std::cout << "reject by fullmatch difference!!  " << node_id << "  " << submap_id << std::endl;
      std::cout << "differ : "
                << (globalPose2dOfNode.inverse()*node_global_pose_submap_origin).
                   translation().norm() << "  " 
                << (globalPose2dOfNode.inverse()*node_global_pose_submap_origin).normalized_angle()  
                << std::endl;
      return;
    }
    else
    {
      std::cout << "accept by fullmatch difference!!  " << node_id << "  " << submap_id << std::endl;
      std::cout << "differ : "
                << (globalPose2dOfNode.inverse()*node_global_pose_submap_origin).
                   translation().norm()
                << std::endl;
    }
  }



  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;
  if (firstTime)
  {
    constraint->reset(new Constraint{submap_id,
                                     node_id,
                                     {transform::Embed3D(constraint_transform),
                                      10000.0,
                                      110000.0},
                                     Constraint::INTER_SUBMAP});
    firstTime= false;
  }
  else
  {
    constraint->reset(new Constraint{submap_id,
                                     node_id,
                                     {transform::Embed3D(constraint_transform),
                                      options_.loop_closure_translation_weight(), //*0.1
                                      options_.loop_closure_rotation_weight()},  //*0.1
                                     Constraint::INTER_SUBMAP});
  }


  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;

    info << " matches";

    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}



void ConstraintBuilder2D::RunWhenDoneCallback() {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    absl::MutexLock locker(&mutex_);
    CHECK(when_done_ != nullptr);
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }
    if (options_.log_matches()) {
      LOG(INFO) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.";
      LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
    }
    constraints_.clear();
    callback = std::move(when_done_);
    when_done_.reset();
    kQueueLengthMetric->Set(constraints_.size());
  }
  (*callback)(result);
}

int ConstraintBuilder2D::GetNumFinishedNodes() {
  absl::MutexLock locker(&mutex_);
  return num_finished_nodes_;
}

void ConstraintBuilder2D::DeleteScanMatcher(const SubmapId& submap_id) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
  per_submap_sampler_.erase(submap_id);
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
}

void ConstraintBuilder2D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "mapping_constraints_constraint_builder_2d_constraints",
      "Constraints computed");
  kConstraintsSearchedMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
  kConstraintsFoundMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "found"}});
  kGlobalConstraintsSearchedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
  kGlobalConstraintsFoundMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "found"}});
  auto* queue_length = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_queue_length", "Queue length");
  kQueueLengthMetric = queue_length->Add({});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "mapping_constraints_constraint_builder_2d_scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric = scores->Add({{"search_region", "local"}});
  kGlobalConstraintScoresMetric = scores->Add({{"search_region", "global"}});
  auto* num_matchers = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_num_submap_scan_matchers",
      "Current number of constructed submap scan matchers");
  kNumSubmapScanMatchersMetric = num_matchers->Add({});
}

void ConstraintBuilder2D::setConstraintMinScore(double val)
{
  constraintBuilderMinScore = val;
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
