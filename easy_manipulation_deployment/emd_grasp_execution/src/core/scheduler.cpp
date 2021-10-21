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
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "emd/grasp_execution/core/scheduler.hpp"

namespace grasp_execution
{

namespace core
{

struct Worker
{
  std::shared_ptr<std::thread> execution_thread;
  std::future<void> execution_future;
};

class WorkflowImplT
{
public:
  WorkflowImplT()
  {
  }

  explicit WorkflowImplT(Scheduler::WorkflowT _workflow)
  : workflow(_workflow)
  {
    sig = std::promise<result_t>();
    future = sig.get_future();
  }

  WorkflowImplT & operator=(const WorkflowImplT &) = delete;

  WorkflowImplT & operator=(WorkflowImplT && rhs) noexcept
  {
    sig = std::move(rhs.sig);
    future = std::move(rhs.future);
    workflow = std::move(rhs.workflow);
    return *this;
  }

  WorkflowImplT(const WorkflowImplT &) = delete;

  WorkflowImplT(WorkflowImplT && rhs) noexcept
  {
    sig = std::move(rhs.sig);
    future = std::move(rhs.future);
    workflow = std::move(rhs.workflow);
  }

  Scheduler::WorkflowT workflow;
  std::promise<result_t> sig;
  std::shared_future<result_t> future;
};

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
    const std::string & workflow_id,
    std::promise<result_t> & workflow_sig,
    std::promise<void> & _sig,
    result_t result);

  size_t concurrency;

  std::mutex metadata_mutex;

  std::deque<WorkflowImplT> workflow_queue;
  std::deque<std::string> id_queue;
  std::unordered_map<std::string, std::shared_future<result_t>> queued_task_map;
  std::unordered_map<std::string, std::shared_future<result_t>> ongoing_task_map;
  std::unordered_map<std::string, result_t> finished_task_map;

  std::vector<Worker> workers;
};

void Scheduler::Impl::add_queue(
  const WorkflowT & workflow,
  const std::string & workflow_id)
{
  workflow_queue.emplace_back(workflow);
  id_queue.push_back(workflow_id);
  queued_task_map[workflow_id] = workflow_queue.back().future;
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


const result_t & Scheduler::get_result(const std::string & workflow_id) const
{
  // Lock scheduler metadata
  std::lock_guard<std::mutex> guard(impl_->metadata_mutex);
  return impl_->finished_task_map.at(workflow_id);
}

void Scheduler::Impl::stop_finished_worker()
{
  for (auto & worker : workers) {
    // Check if there are on going tasks
    if (worker.execution_thread) {
      // Check if the ongoing tasks is finished
      auto status = worker.execution_future.wait_for(std::chrono::nanoseconds(0));
      if (status != std::future_status::ready) {
        continue;
      }
      worker.execution_thread->join();
      worker.execution_thread.reset();
    }
  }
}

void Scheduler::Impl::start_worker(
  size_t worker_id, const WorkflowT & workflow, const std::string && workflow_id)
{
  // Create signal for workcell result
  auto sig = std::promise<void>();

  // Register future for monitoring signal
  workers[worker_id].execution_future = sig.get_future();

  // Add workflow id into the ongoing task map
  WorkflowImplT workflow_impl(workflow);
  ongoing_task_map[workflow_id] = workflow_impl.future;

  // Start execution thread
  workers[worker_id].execution_thread = std::make_shared<std::thread>(
    [this, wf = std::move(workflow_impl)](
      std::promise<void> && _sig,
      const std::string && _workflow_id) mutable
    {
      result_t result = wf.workflow(_workflow_id);

      // Start execution ending callback
      // This will check if there are additional queue item in task
      // TODO(Briancbn): setting workflow prerequisite, (DAG)?
      execution_ending_cb(_workflow_id, wf.sig, _sig, result);
    }, std::move(sig), std::move(workflow_id));
}

void Scheduler::Impl::execution_ending_cb(
  const std::string & workflow_id,
  std::promise<result_t> & workflow_sig,
  std::promise<void> & _sig,
  result_t result)
{
  WorkflowImplT workflow;
  std::string new_workflow_id;
  bool queue = false;
  {    // Lock scheduler metadata
    std::lock_guard<std::mutex> guard(metadata_mutex);

    workflow_sig.set_value(result);

    // Add workflow id and the respecting result to the finished task map
    finished_task_map[workflow_id] = result;

    // Remove workflow id from the ongoing task map
    ongoing_task_map.erase(workflow_id);

    if (!id_queue.empty()) {
      queue = true;

      // TODO(anyone): setting workflow prerequisite, (DAG)?
      // Select the first workflow and workflow_id in queue
      new_workflow_id = std::move(id_queue.front());
      id_queue.pop_front();
      workflow = std::move(workflow_queue.front());
      workflow_queue.pop_front();

      // Add workflow id into the ongoing task map
      ongoing_task_map[new_workflow_id] = std::move(queued_task_map[new_workflow_id]);
      queued_task_map.erase(new_workflow_id);
    } else {
      // workflow done, expose result end recursive function
      _sig.set_value();
      return;
    }
  }    // Unlock scheduler metadata

  if (queue) {
    // Start the next workflow
    workflow.workflow(new_workflow_id);

    // Start recursive callback
    execution_ending_cb(new_workflow_id, workflow.sig, _sig, true);
  }
}

Scheduler::Scheduler(size_t concurrency)
: impl_(std::make_unique<Impl>(concurrency))
{
}

Scheduler::~Scheduler()
{
  // Cancel all workflows in queue
  for (auto & workflow_id : impl_->id_queue) {
    cancel_workflow(workflow_id);
  }
}

Workflow::Status Scheduler::add_workflow(
  const std::string & workflow_id,
  WorkflowT workflow)
{
  // Doesn't allow thread exit, during function call
  {   // Lock scheduler metadata
    std::lock_guard<std::mutex> guard(impl_->metadata_mutex);

    // Check if the workflow_id exist in history
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

    impl_->stop_finished_worker();

    int worker_id = impl_->get_available_worker();

    if (worker_id == -1) {
      // Check if task is already in queue
      if (impl_->queued_task_map.find(workflow_id) != impl_->queued_task_map.end()) {
        return Workflow::Status::INVALID;
      }
      impl_->add_queue(workflow, workflow_id);
      return Workflow::Status::QUEUED;
    } else {
      impl_->start_worker(worker_id, workflow, std::move(workflow_id));
      return Workflow::Status::ONGOING;
    }
  }   // Unlock scheduler metadata
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
      impl_->queued_task_map.erase(workflow_id);
      return Workflow::Status::CANCELLED;
    }
  }

  // Cannot find workflow_id anywhere
  return Workflow::Status::INVALID;
}

void Scheduler::wait_till_all_complete() const
{
  for (auto & worker : impl_->workers) {
    if (worker.execution_future.valid()) {
      worker.execution_future.wait();
    }
  }
}


Workflow::Status Scheduler::wait_till_complete(
  const std::string & workflow_id,
  result_t & result) const
{
  std::shared_future<result_t> future;
  switch (get_status(workflow_id)) {
    case Workflow::Status::COMPLETED:
      result = impl_->finished_task_map[workflow_id];
      break;
    case Workflow::Status::ONGOING:
      future = impl_->ongoing_task_map[workflow_id];
      future.wait();
      result = get_result(workflow_id);
      break;
    case Workflow::Status::QUEUED:
      future = impl_->queued_task_map[workflow_id];
      future.wait();
      result = get_result(workflow_id);
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
  if (impl_->queued_task_map.find(workflow_id) != impl_->queued_task_map.end()) {
    return Workflow::Status::QUEUED;
  }

  // Cannot find workflow_id anywhere
  return Workflow::Status::INVALID;
}

}  // namespace core

}  // namespace grasp_execution
