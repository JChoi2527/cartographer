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

#include "cartographer/mapping/internal/motion_filter.h"

#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::MotionFilterOptions options;
  options.set_max_time_seconds(
      parameter_dictionary->GetDouble("max_time_seconds"));
  options.set_max_distance_meters(
      parameter_dictionary->GetDouble("max_distance_meters"));
  options.set_max_angle_radians(
      parameter_dictionary->GetDouble("max_angle_radians"));
  return options;
}

MotionFilter::MotionFilter(const proto::MotionFilterOptions& options)
    : options_(options) {}

bool MotionFilter::IsSimilar(const common::Time time,
                             const transform::Rigid3d& pose) {
  LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
      << "Motion filter reduced the number of nodes to "
      << 100. * num_different_ / num_total_ << "%.";
  ++num_total_;
  std::chrono::duration<double> chk_seconds = common::FromSeconds(options_.max_time_seconds());

    if(MatchSubmap::getFirstMatchSubmap() == false) {
    // std::cout << "FULL MATCH || OR LOCAL MATCHED NOT YET" << std::endl;
    // std::cout << "set time: 0.2 seconds" << std::endl;
    chk_seconds = common::FromSeconds(0.2);
  } else {
    bool res = this->matchSubmap.delayTime();
    if(res == false) {
        // std::cout << "FULL MATCHED!! or LOCAL MATCHED BUT!!" << std::endl;
        // std::cout << "set time: 0.2 seconds" << std::endl;
        // std::cout << "Need more 30 seconds..!!" << std::endl;
        chk_seconds = common::FromSeconds(0.2);
    }
  }

  if (num_total_ > 1 &&
      time - last_time_ <= chk_seconds &&
      (pose.translation() - last_pose_.translation()).norm() <=
          options_.max_distance_meters() &&
      transform::GetAngle(pose.inverse() * last_pose_) <=
          options_.max_angle_radians() ) {
    return true;
  }
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}

}  // namespace mapping
}  // namespace cartographer
