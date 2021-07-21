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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "emd/grasp_execution/dynamic_safety/collision_checker.hpp"
#include "moveit/robot_state/conversions.h"

namespace grasp_execution
{

namespace dynamic_safety
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dynamic_safety.collision checker");

class CollisionChecker::Context
{
public:
  // cppcheck-suppress unknownMacro
  RCLCPP_SMART_PTR_DEFINITIONS(Context)

  static Context::UniquePtr create(
    const planning_scene::PlanningScenePtr & context,
    const std::string & detector_group)
  {
    Context::UniquePtr new_context;
    new_context.reset(new Context());

    // Deep copy planning scene
    new_context->scene_ =
      planning_scene::PlanningScene::clone(context);

    // Reconstruct acm
    new_context->acm_.reset(new collision_detection::AllowedCollisionMatrix());

    const auto & rm = new_context->scene_->getRobotModel();
    // Use default collision operations in the SRDF to setup the acm
    const std::vector<std::string> & collision_links =
      rm->getLinkModelNamesWithCollisionGeometry();
    new_context->acm_->setEntry(collision_links, collision_links, false);

    // allow collisions for pairs that have been disabled
    const std::vector<srdf::Model::DisabledCollision> & dc =
      rm->getSRDF()->getDisabledCollisionPairs();
    for (const srdf::Model::DisabledCollision & it : dc) {
      new_context->acm_->setEntry(it.link1_, it.link2_, true);
    }
    // new_context->scene_->getAllowedCollisionMatrixNonConst() =
    //    *new_context->acm_;

    new_context->detector_group_ = detector_group;

    return new_context;
  }

  // static Context::UniquePtr create(
  //   const moveit_msgs::msg::PlanningScene::SharedPtr & context,
  //   const std::string & detector_group) {
  // }

  static Context::UniquePtr clone(
    const Context::SharedPtr & context)
  {
    Context::UniquePtr cloned_context;
    cloned_context.reset(new Context());

    // Deep copy planning scene
    cloned_context->scene_ =
      planning_scene::PlanningScene::clone(context->scene_);

    cloned_context->detector_group_ = context->detector_group_;
    return cloned_context;
  }

  void configure(const Option & option)
  {
    collision_request_.group_name = detector_group_;
    collision_request_.distance = option.distance;
    collision_request_.contacts = true;
  }

  void run(const moveit::core::RobotState & state, uint8_t & result, double & distance)
  {
    auto & current_state = scene_->getCurrentStateNonConst();
    for (auto & name :
      state.getJointModelGroup(collision_request_.group_name)->getVariableNames())
    {
      current_state.setJointPositions(name, state.getJointPositions(name));
    }
    // Check robot collision
    current_state.updateCollisionBodyTransforms();
    // scene_->checkCollision(collision_request_, collision_result_, state);
    result = false;
    collision_result_.clear();
    scene_->getCollisionEnv()->checkRobotCollision(
      collision_request_, collision_result_, current_state);
    distance = collision_result_.distance;
    result |= collision_result_.collision;
    collision_result_.print();

    collision_result_.clear();

    scene_->getCollisionEnvUnpadded()->checkSelfCollision(
      collision_request_, collision_result_, current_state, *acm_);

    distance = collision_result_.distance;
    result |= collision_result_.collision;
    collision_result_.print();

    collision_result_.clear();
  }

  void update(const moveit_msgs::msg::RobotState & state_msg)
  {
    scene_->setCurrentState(state_msg);
  }

  Context(const Context & other) = default;

  ~Context() {}

private:
  Context()
  {
  }

  std::string detector_group_;

  planning_scene::PlanningScenePtr scene_;

  // collision request
  collision_detection::CollisionRequest collision_request_;
  collision_detection::CollisionResult collision_result_;
  collision_detection::AllowedCollisionMatrixPtr acm_;
};

CollisionChecker::CollisionChecker()
: started_(false)
{
}

CollisionChecker::~CollisionChecker()
{
  reset();
}

void CollisionChecker::configure(
  const Option & option,
  const planning_scene::PlanningScenePtr & scene,
  const robot_trajectory::RobotTrajectoryPtr & rt)
{
  // Create the vector of states based on the resolution set when discrete.
  if (!option.continuous) {
    step_ = option.step;

    // Clear everything
    states_.clear();
    duration_from_start_.clear();
    results_.clear();
    distances_.clear();

    double full_duration = rt->getDuration();
    int state_size = static_cast<int>(full_duration / option.step);
    auto temp_state = std::make_shared<moveit::core::RobotState>(scene->getRobotModel());
    double start_offset = full_duration - static_cast<double>(state_size) * option.step;
    for (int i = 0; i < state_size; i++) {
      duration_from_start_.push_back(start_offset + static_cast<double>(i) * option.step);
      rt->getStateAtDurationFromStart(duration_from_start_.back(), temp_state);
      states_.push_back(*temp_state);
    }
    // resize result
    results_.resize(state_size, false);
    distances_.resize(state_size, -1);

    // Clear everything
    contexts_.clear();
    runners_.clear();
    // runner_ms_.clear();
    // runner_cvs_.clear();
    // runner_flags_.clear();

    // Setup synchronization variables
    started_ = true;

    // {
    //   std::vector<std::mutex> temp_mutex(option.thread_count);
    //   runner_ms_.swap(temp_mutex);

    //   std::deque<std::atomic_int> temp_flags_(option.thread_count);
    //   runner_flags_.swap(temp_flags_);
    // }

    // Start context and runners
    thread_count_ = option.thread_count;
    current_iteration_ = 0;
    for (int i = 0; i < option.thread_count; i++) {
      auto context = Context::create(scene, rt->getGroupName());
      context->configure(option);
      contexts_.push_back(std::move(context));
      runners_.emplace_back(
        std::make_shared<std::thread>(
          [ = ]() -> void
          {
            _discrete_runner_fn(i);
          }));
      // runner_cvs_.emplace_back(std::make_unique<std::condition_variable>());

      // runner_flags_[i] = false;

      // Setup realtime sched
      if (option.realtime) {
        struct sched_param param;
        param.sched_priority = 99;
        pthread_setschedparam(runners_[i]->native_handle(), SCHED_FIFO, &param);
      }
    }
  }
}

void CollisionChecker::update(
  const moveit::core::RobotState & state)
{
  moveit_msgs::msg::RobotState state_msgs;
  moveit::core::robotStateToRobotStateMsg(state, state_msgs);
  for (auto & context : contexts_) {
    context->update(state_msgs);
  }
}

void CollisionChecker::run_once(
  double point,
  double look_ahead_time,
  double & collision_point)
{
  if (!started_ || duration_from_start_.empty()) {
    return;
  }
  itr_ = static_cast<int>((point - duration_from_start_.front()) / step_);
  itr_end_ = static_cast<int>((point + look_ahead_time - duration_from_start_.front()) / step_);
  // end should not exceed trajectory max
  if (itr_end_ > static_cast<int>(states_.size())) {
    itr_end_ = static_cast<int>(states_.size());
  }

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
  for (size_t idx = 1; idx < results_.size(); idx++) {
    if (results_[idx]) {
      collision_point = duration_from_start_[idx - 1];
      break;
    }
  }
}

void CollisionChecker::reset()
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

void CollisionChecker::update_traj(
  const robot_trajectory::RobotTrajectoryPtr & rt,
  const Option & option)
{
  // Clear everything
  states_.clear();
  duration_from_start_.clear();
  results_.clear();
  distances_.clear();

  double full_duration = rt->getDuration();
  int state_size = static_cast<int>(full_duration / option.step);
  auto temp_state = std::make_shared<moveit::core::RobotState>(rt->getRobotModel());
  double start_offset = full_duration - static_cast<double>(state_size) * option.step;
  for (int i = 0; i < state_size; i++) {
    duration_from_start_.push_back(start_offset + static_cast<double>(i) * option.step);
    rt->getStateAtDurationFromStart(duration_from_start_.back(), temp_state);
    states_.push_back(*temp_state);
  }
  // resize result
  results_.resize(state_size, false);
  distances_.resize(state_size, -1);
}

void CollisionChecker::_discrete_runner_fn(int runner_id)
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
    int itr;
    // Unlock immediately to start the rest of the thread
    while (true) {
      itr = static_cast<int>(itr_++);
      if (itr >= static_cast<int>(itr_end_)) {
        break;
      }
      contexts_[runner_id]->run(states_[itr], results_[itr], distances_[itr]);
    }

    runner_lk.lock();
    if (--n_active_workers_ == 0) {
      runner_lk.unlock();
      init_cv_.notify_all();
    }
  }
}

}  // namespace dynamic_safety

}  // namespace grasp_execution
