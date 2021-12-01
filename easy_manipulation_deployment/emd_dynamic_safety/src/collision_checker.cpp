// Copyright 2021 ROS Industrial Consortium Asia Pacific
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "emd/dynamic_safety/collision_checker.hpp"
#include "emd/interpolate.hpp"
#include "emd/profiler.hpp"

#ifdef EMD_DYNAMIC_SAFETY_TESSERACT
#include "emd/dynamic_safety/collision_checker_tesseract.hpp"
#endif  // EMD_DYNAMIC_SAFETY_TESSERACT

#ifdef EMD_DYNAMIC_SAFETY_MOVEIT
#include "emd/dynamic_safety/collision_checker_moveit.hpp"
#endif  // EMD_DYNAMIC_SAFETY_MOVEIT

namespace dynamic_safety
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dynamic_safety.collision_checker");

class CollisionChecker::Impl
{
public:
  Impl() = default;

  virtual ~Impl() = default;

  void configure(
    const CollisionCheckerOption & option,
    const std::string & robot_urdf,
    const std::string & robot_srdf);

  void add_trajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt);

  void update(const sensor_msgs::msg::JointState & state);

  void update(const moveit_msgs::msg::PlanningScene & scene_msg);

  void run_once(
    double current_time,
    double look_ahead_time,
    double & collision_time);

  void sample_typical_time_point(
    int sample_size,
    double look_ahead_time,
    std::vector<double> & time_point_samples);

  void stop();

  void reset();

protected:
  // Main function for running discrete collision checking
  void _runner_fn(int runner_id, bool continuous);

  double step_;
  std::vector<std::unique_ptr<CollisionCheckerContext>> contexts_;

  trajectory_msgs::msg::JointTrajectory trajectory_;

  // Duplicated trajectory to avoid data racing in continuous collision checking
  trajectory_msgs::msg::JointTrajectory trajectory2_;

  // Run result
  // cannot use bool https://stackoverflow.com/a/25194424
  // also bool is not thread safe
  std::vector<uint8_t> results_;
  std::vector<double> distances_;

  double start_offset_;
  bool continuous_;

  // thread handling / Synchronization variables
  // Similar to https://stackoverflow.com/a/53274193/13517633
  std::vector<std::shared_ptr<std::thread>> runners_;
  std::condition_variable init_cv_;
  std::mutex init_m_;
  int n_active_workers_;
  int current_iteration_;
  int thread_count_;

  // Atomic variables to monitor collision checking progress
  std::atomic_int itr_;
  std::atomic_int itr_end_;

  // Atomic variables to control thread starting and ending
  std::atomic_bool started_;
};

void CollisionChecker::Impl::configure(
  const CollisionCheckerOption & option,
  const std::string & robot_urdf,
  const std::string & robot_srdf)
{
  contexts_.clear();

  // Create the vector of states based on the resolution set when discrete.
  continuous_ = option.continuous;
  step_ = option.step;

  // Clear everything
  contexts_.clear();
  runners_.clear();

  // Setup synchronization variables
  started_ = true;

  // Start context and runners
  thread_count_ = option.thread_count;
  current_iteration_ = 0;
  for (size_t i = 0; i < static_cast<size_t>(option.thread_count); i++) {
    std::unique_ptr<CollisionCheckerContext> context;
    if (option.framework == "moveit") {
#ifdef EMD_DYNAMIC_SAFETY_MOVEIT
      context = std::make_unique<dynamic_safety_moveit::MoveitCollisionCheckerContext>(
        robot_urdf, robot_srdf, option.collision_checking_plugin);
#else
      RCLCPP_ERROR(LOGGER, "Framework %s not defined", option.framework.c_str());
      // TODO(anyone): exception handling
      return;
#endif
    } else if (option.framework == "tesseract") {
#ifdef EMD_DYNAMIC_SAFETY_TESSERACT
      context = std::make_unique<dynamic_safety_tesseract::TesseractCollisionCheckerContext>(
        robot_urdf, robot_srdf, option.collision_checking_plugin);
#else
      RCLCPP_ERROR(LOGGER, "Framework %s not defined", option.framework.c_str());
      // TODO(anyone): exception handling
      return;
#endif
    }
    context->configure(option);
    contexts_.push_back(std::move(context));
    runners_.emplace_back(
      std::make_shared<std::thread>(
        [ = ]() -> void
        {
          _runner_fn(static_cast<int>(i), continuous_);
        }));

    // Setup realtime sched
    if (option.realtime) {
      struct sched_param param;
      param.sched_priority = 99;
      pthread_setschedparam(runners_[i]->native_handle(), SCHED_FIFO, &param);
    }
  }
}

void CollisionChecker::Impl::add_trajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
{
  if (rt->points.empty()) {
    RCLCPP_ERROR(LOGGER, "Trajectory Empty");
    // TODO(Briancbn): Proper exception handling.
    return;
  }

  trajectory_.joint_names = rt->joint_names;

  // Clear everything
  trajectory_.points.clear();
  results_.clear();
  distances_.clear();

  // Re-time trajectory
  double full_duration = rclcpp::Duration(rt->points.back().time_from_start).seconds();
  int state_size = static_cast<int>(full_duration / step_);
  // record the starting offset
  start_offset_ = full_duration - static_cast<double>(state_size) * step_;
  double time_from_start = start_offset_;

  size_t num_points = rt->points.size();
  size_t before, after;
  size_t i = 0;
  for (int idx = 0; idx <= state_size; idx++) {
    for (; i < rt->points.size(); i++) {
      if (rclcpp::Duration(rt->points[i].time_from_start).seconds() >= time_from_start) {
        break;
      }
    }
    before = std::max<size_t>((i == 0) ? 0 : (i - 1), 0);  // Avoid unsigned int 0 minus 1
    after = std::min<size_t>(i, num_points - 1);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
    emd::core::interpolate_between_points(
      rt->points[before].time_from_start, rt->points[before],
      rt->points[after].time_from_start, rt->points[after],
      point.time_from_start, point);
    trajectory_.points.push_back(point);
    time_from_start += step_;
  }

  // Duplicated trajectory to avoid data racing in continuous collision checking
  if (continuous_) {
    trajectory2_ = trajectory_;
  }

  // resize result
  results_.resize(trajectory_.points.size(), false);
  distances_.resize(trajectory_.points.size(), -1);
}

void CollisionChecker::Impl::update(const sensor_msgs::msg::JointState & state)
{
  for (auto & context : contexts_) {
    context->update(state);
  }
}

void CollisionChecker::Impl::update(const moveit_msgs::msg::PlanningScene & scene_msg)
{
  for (auto & context : contexts_) {
    context->update(scene_msg);
  }
}

void CollisionChecker::Impl::_runner_fn(int runner_id, bool continuous)
{
  int next_iteration = 1;
  while (true) {
    std::unique_lock<std::mutex> runner_lk(init_m_);
    init_cv_.wait(
      runner_lk,
      [&next_iteration, & current_iteration_ = current_iteration_]
      {return current_iteration_ == next_iteration;});
    runner_lk.unlock();

    ++next_iteration;

    if (!started_) {
      return;
    }

    // ============= runner started ===============
    // Move this to attributes?
    size_t itr;
    size_t itr2;
    // Unlock immediately to start the rest of the thread
    while (true) {
      itr = static_cast<size_t>(itr_++);
      size_t itr_end = static_cast<size_t>(itr_end_);
      if (itr + 1 > itr_end) {
        if (continuous) {
          // Continuous collision checking duplicate last point
          itr2 = itr;
        }
        if (itr > itr_end) {
          // Break when reaches the last point
          break;
        }
      } else {
        itr2 = itr + 1;
      }
      size_t runner_id_u = static_cast<size_t>(runner_id);
      if (continuous) {
        contexts_[runner_id_u]->run_continuous(
          trajectory_.joint_names, trajectory_.points[itr],
          trajectory2_.points[itr2], results_[itr], distances_[itr]);
      } else {
        contexts_[runner_id_u]->run_discrete(
          trajectory_.joint_names, trajectory_.points[itr], results_[itr], distances_[itr]);
      }
    }

    runner_lk.lock();
    if (--n_active_workers_ == 0) {
      runner_lk.unlock();
      init_cv_.notify_all();
    }
  }
}

void CollisionChecker::Impl::run_once(
  double current_time,
  double look_ahead_time,
  double & collision_time)
{
  if (!started_ || trajectory_.points.empty()) {
    // TODO(Briancbn): proper exception handling
    return;
  }

  int start_index = static_cast<int>((current_time - start_offset_) / step_);
  int end_index = static_cast<int>((current_time + look_ahead_time - start_offset_) / step_);
  // end should not exceed trajectory max
  if (start_index < 0) {
    start_index = 0;
  }

  start_index = std::min<int>(start_index, static_cast<int>(trajectory_.points.size()) - 1);
  end_index = std::min<int>(end_index, static_cast<int>(trajectory_.points.size()) - 1);

  itr_ = start_index;
  itr_end_ = end_index;
  {
    std::lock_guard<std::mutex> lk(init_m_);
    n_active_workers_ = thread_count_;
    current_iteration_++;
  }

  // start all threads
  init_cv_.notify_all();

  {
    std::unique_lock<std::mutex> lk(init_m_);
    init_cv_.wait(
      lk, [ & n_active_workers_ = n_active_workers_]
      {return n_active_workers_ == 0;});
  }

  // Ignore the first index, cuz ... cannot avoid..
  // TODO(anyone): fix the first index
  collision_time = -1;
  if (results_[static_cast<size_t>(start_index)]) {
    collision_time = start_offset_ + step_ * start_index;
  } else {
    for (int idx = start_index + 1; idx <= end_index; idx++) {
      if (results_[static_cast<size_t>(idx)]) {
        collision_time = start_offset_ + (idx - 1) * step_;
        break;
      }
    }
  }
}

void CollisionChecker::Impl::sample_typical_time_point(
  int sample_size,
  double look_ahead_time,
  std::vector<double> & time_point_samples)
{
  double range =
    rclcpp::Duration(trajectory_.points.back().time_from_start).seconds() - look_ahead_time;
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  time_point_samples.clear();
  for (int i = 0; i < sample_size; i++) {
    time_point_samples.push_back(static_cast<double>(std::rand()) / RAND_MAX * range);
  }
}

void CollisionChecker::Impl::stop()
{
}

void CollisionChecker::Impl::reset()
{
  started_ = false;
  {
    std::lock_guard<std::mutex> lk(init_m_);
    current_iteration_++;
  }
  RCLCPP_INFO(LOGGER, "Send stopping signal");
  init_cv_.notify_all();
  RCLCPP_INFO(LOGGER, "Joining runners.");
  for (auto & runner : runners_) {
    if (runner->joinable()) {
      runner->join();
    }
  }
}

CollisionChecker::CollisionChecker()
: impl_ptr_(std::make_unique<Impl>())
{
}

CollisionChecker::~CollisionChecker()
{
  reset();
}

void CollisionChecker::configure(
  const CollisionCheckerOption & option,
  const std::string & robot_urdf,
  const std::string & robot_srdf)
{
  impl_ptr_->configure(option, robot_urdf, robot_srdf);
}

void CollisionChecker::update(
  const sensor_msgs::msg::JointState & state)
{
  impl_ptr_->update(state);
}

void CollisionChecker::update(
  const moveit_msgs::msg::PlanningScene & scene_msg)
{
  impl_ptr_->update(scene_msg);
}

void CollisionChecker::run_once(
  double current_time,
  double look_ahead_time,
  double & collision_time)
{
  impl_ptr_->run_once(current_time, look_ahead_time, collision_time);
}

double CollisionChecker::polling(
  double look_ahead_time,
  int sample_size)
{
  std::vector<double> sample_time_points;
  impl_ptr_->sample_typical_time_point(sample_size, look_ahead_time, sample_time_points);
  double collision_time;
  double collision_checking_duration = 0;
  emd::TimeProfiler<> poller(static_cast<size_t>(sample_size));
  for (auto & start_time : sample_time_points) {
    poller.reset();
    run_once(start_time, look_ahead_time, collision_time);
    collision_checking_duration += poller.lapse_and_record();
  }
  std::ostringstream oss;
  poller.print(oss);
  RCLCPP_INFO(LOGGER, oss.str().c_str());
  return collision_checking_duration / sample_size;
}

void CollisionChecker::reset()
{
  impl_ptr_->reset();
}

void CollisionChecker::stop()
{
  impl_ptr_->stop();
}

void CollisionChecker::add_trajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
{
  impl_ptr_->add_trajectory(rt);
}


}  // namespace dynamic_safety
