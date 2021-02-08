// Copyright 2020 ROS Industrial Consortium Asia Pacific
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

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "grasp_execution/scheduler.hpp"

namespace grasp_execution
{

class Scheduler::Impl
{
public:
  /// \brief Constructor
  explicit Impl(size_t _concurrency)
  : concurrency(_concurrency)
  {
    workers.resize(_concurrency);

    // Clear queue
    workflow_queue.clear();
    id_queue.clear();

    // Clear ongoing tasks;
    ongoing_task_map.clear();
    finished_task_map.clear();
  }

  /// \brief Constructor
  ~Impl()
  {
    for (auto & worker : workers) {
      if (worker.execution_thread) {
        worker.execution_thread->join();
      }
    }
  }

  void stop_finished_worker();

  void start_worker(
    size_t worker_id,
    const WorkflowT & workflow,
    const std::string && task_id);

  void add_queue(
    const WorkflowT & workflow,
    const std::string & workflow_id);

  int get_available_worker() const;

  void execution_ending_cb(
    size_t worker_id,
    std::promise<result_t> & _sig);

  size_t concurrency;

  std::mutex metadata_mutex;

  std::deque<WorkflowT> workflow_queue;
  std::deque<std::string> id_queue;
  std::unordered_map<std::string, size_t> ongoing_task_map;
  std::unordered_map<std::string, result_t> finished_task_map;

  std::vector<Worker> workers;
};

void Scheduler::Impl::add_queue(
  const WorkflowT & workflow,
  const std::string & workflow_id)
{
  std::lock_guard<std::mutex> guard(metadata_mutex);
  workflow_queue.push_back(workflow);
  id_queue.push_back(workflow_id);
}

int Scheduler::Impl::get_available_worker() const
{
  for (size_t i = 0; i < concurrency; i++) {
    if (!workers[i].execution_thread) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void Scheduler::Impl::stop_finished_worker()
{
  for (auto & worker : workers) {
    // Check if there are on going tasks
    if (worker.execution_thread) {
      // Check if the ongoing tasks is finished
      auto status = worker.execution_future.wait_for(std::chrono::nanoseconds(0));
      if (status == std::future_status::ready) {
        worker.execution_thread->join();
        worker.execution_thread.reset();
      }
    }
  }
}

void Scheduler::Impl::start_worker(
  size_t worker_id, const WorkflowT & workflow, const std::string && workflow_id)
{
  // Create signal for workcell result
  auto sig = std::promise<result_t>();

  // Register future for monitoring signal
  workers[worker_id].execution_future = sig.get_future();

  // Start execution thread
  workers[worker_id].execution_thread = std::make_shared<std::thread>(
    [ = ](std::promise<result_t> && _sig, const std::string && _workflow_id) {
      {    // Lock scheduler metadata
        std::lock_guard<std::mutex> guard(metadata_mutex);

        // Add workflow id into the ongoing task map
        ongoing_task_map[_workflow_id] = worker_id;
      }    // Unlock scheduler metadata

      result_t result = workflow(_workflow_id);

      {    // Lock scheduler metadata
        std::lock_guard<std::mutex> guard(metadata_mutex);

        // Remove workflow id from the ongoing task map
        ongoing_task_map.erase(_workflow_id);

        // Add workflow id to finished task
        finished_task_map[_workflow_id] = result;
      }    // Unlock scheduler metadata

      // workflow done, expose result
      _sig.set_value(result);

      // Start execution ending callback
      // This will check if there are additional queue item in task
      // TODO(Briancbn): setting workflow prerequisite, (DAG)?
      execution_ending_cb(worker_id, _sig);
    }, std::move(sig), std::move(workflow_id));
}

void Scheduler::Impl::execution_ending_cb(
  size_t worker_id,
  std::promise<result_t> & _sig)
{
  WorkflowT workflow;
  std::string workflow_id;
  bool queue = false;
  {    // Lock scheduler metadata
    std::lock_guard<std::mutex> guard(metadata_mutex);
    if (!id_queue.empty()) {
      queue = true;

      // reset signal for workcell result
      // TODO(anyone): inspect mutex implemetation?
      _sig = std::promise<result_t>();

      // Register future for monitoring signal
      workers[worker_id].execution_future = _sig.get_future();

      // TODO(anyone): setting workflow prerequisite, (DAG)?
      // Select the first workflow and workflow_id in queue
      workflow_id = std::move(id_queue.front());
      id_queue.pop_front();
      workflow = std::move(workflow_queue.front());
      workflow_queue.pop_front();

      // Add workflow id into the ongoing task map
      ongoing_task_map[workflow_id] = worker_id;
    }
  }    // Unlock scheduler metadata

  if (queue) {
    // Start the next workflow
    result_t result = workflow(workflow_id);

    {    // Lock scheduler metadata
      std::lock_guard<std::mutex> guard(metadata_mutex);

      // Remove workflow id from the ongoing task map
      ongoing_task_map.erase(workflow_id);

      // Add workflow id to finished task
      finished_task_map[workflow_id] = result;
    }    // Unlock scheduler metadata

    // workflow done, expose result
    _sig.set_value(result);

    // Start recursive callback
    execution_ending_cb(worker_id, _sig);
  }
}

Scheduler::Scheduler(size_t concurrency)
: impl_(std::make_unique<Impl>(concurrency))
{
}

Scheduler::~Scheduler()
{
}

Workflow::Status Scheduler::add_workflow(
  const std::string & workflow_id,
  WorkflowT workflow)
{
  // Check if the workflow_id exist in history
  {   // Lock scheduler metadata
    std::lock_guard<std::mutex> guard(impl_->metadata_mutex);
    if (impl_->finished_task_map.find(workflow_id) !=
      impl_->finished_task_map.end())
    {
      return Workflow::Status::INVALID;
    }
  }   // Unlock scheduler metadata

  impl_->stop_finished_worker();

  int worker_id = impl_->get_available_worker();

  if (worker_id == -1) {
    // Check if task is already in queue
    // TODO(anyone): better way to check queue
    for (auto & queue_id : impl_->id_queue) {
      if (queue_id == workflow_id) {
        return Workflow::Status::INVALID;
      }
    }
    impl_->add_queue(workflow, workflow_id);
    return Workflow::Status::QUEUED;
  } else {
    impl_->start_worker(worker_id, workflow, std::move(workflow_id));
    return Workflow::Status::ONGOING;
  }
}

Workflow::Status Scheduler::cancel_workflow(
  const std::string & workflow_id)
{
  // Check if the workflow_id exist in completed history
  std::lock_guard<std::mutex> guard(impl_->metadata_mutex);
  if (impl_->finished_task_map.find(workflow_id) !=
    impl_->finished_task_map.end())
  {
    return Workflow::Status::INVALID;
  }

  // Check if the workflow_id is already on-going
  if (impl_->ongoing_task_map.find(workflow_id) !=
    impl_->ongoing_task_map.end())
  {
    return Workflow::Status::INVALID;
  }

  // Check if workflow_id is in queue
  // TODO(anyone): better way to check queue
  for (size_t i = 0; i < impl_->id_queue.size(); i++) {
    // Remove workflow id
    if (impl_->id_queue[i] == workflow_id) {
      impl_->id_queue.erase(impl_->id_queue.begin() + i);
      impl_->workflow_queue.erase(impl_->workflow_queue.begin() + i);
      return Workflow::Status::CANCELLED;
    }
  }

  // Cannot find workflow_id anywhere
  return Workflow::Status::INVALID;
}

void Scheduler::wait_till_all_complete() const
{
  while (!impl_->id_queue.empty()) {
    for (auto & worker : impl_->workers) {
      worker.execution_future.wait();
    }
  }
}


Workflow::Status Scheduler::wait_till_complete(
  const std::string & workflow_id,
  result_t & result) const
{
  switch (get_status(workflow_id)) {
    case Workflow::Status::COMPLETED:
      result = impl_->finished_task_map[workflow_id];
      break;
    case Workflow::Status::ONGOING:
      result =
        impl_->workers[impl_->ongoing_task_map[workflow_id]].execution_future.get();
      break;
    default:
      return Workflow::Status::INVALID;
  }
  return Workflow::Status::COMPLETED;
}

Workflow::Status Scheduler::get_status(
  const std::string & workflow_id) const
{
  // Check if the workflow_id exist in completed history
  std::lock_guard<std::mutex> guard(impl_->metadata_mutex);
  if (impl_->finished_task_map.find(workflow_id) !=
    impl_->finished_task_map.end())
  {
    return Workflow::Status::COMPLETED;
  }

  // Check if the workflow_id is already on-going
  if (impl_->ongoing_task_map.find(workflow_id) !=
    impl_->ongoing_task_map.end())
  {
    return Workflow::Status::ONGOING;
  }

  // Check if workflow_id is in queue
  // TODO(anyone): better way to check queue
  for (size_t i = 0; i < impl_->id_queue.size(); i++) {
    // Remove workflow id
    if (impl_->id_queue[i] == workflow_id) {
      return Workflow::Status::QUEUED;
    }
  }

  // Cannot find workflow_id anywhere
  return Workflow::Status::INVALID;
}

}  // namespace grasp_execution
