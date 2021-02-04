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
#include <utility>
#include <vector>

#include "grasp_execution/scheduler.hpp"

namespace grasp_execution
{

class Scheduler::Impl
{
public:
  explicit Impl(size_t _concurrency)
  : concurrency(_concurrency)
  {
    workers.resize(_concurrency);
    workflow_queue.clear();
  }

  ~Impl()
  {
    for (auto & worker : workers) {
      if (worker.execution_thread) {
        worker.execution_thread->join();
      }
    }
  }

  void stop_finished_worker();

  void start_worker(size_t worker_id, const WorkflowT & workflow);

  void add_queue(const WorkflowT & workflow);

  int get_available_worker() const;

  void execution_ending_cb();

  size_t concurrency;

  std::mutex wf_q_mutex;

  std::deque<WorkflowT> workflow_queue;

  std::vector<Worker> workers;
};

void Scheduler::Impl::add_queue(const WorkflowT & workflow)
{
  std::lock_guard<std::mutex> guard(wf_q_mutex);
  workflow_queue.push_back(workflow);
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

void Scheduler::Impl::start_worker(size_t worker_id, const WorkflowT & workflow)
{
  auto sig = std::promise<bool>();
  workers[worker_id].execution_future = sig.get_future();
  workers[worker_id].execution_thread = std::make_shared<std::thread>(
    [ = ](std::promise<bool> && _sig) {
      workflow();
      execution_ending_cb();
      _sig.set_value(true);
    }, std::move(sig));
}

void Scheduler::Impl::execution_ending_cb()
{
  WorkflowT workflow;
  bool queue = false;
  {
    std::lock_guard<std::mutex> guard(wf_q_mutex);
    if (!workflow_queue.empty()) {
      queue = true;
      workflow = std::move(workflow_queue.front());
      workflow_queue.pop_front();
    }
  }
  if (queue) {
    workflow();
    execution_ending_cb();
  }
}

Scheduler::Scheduler(size_t concurrency)
: impl_(std::make_unique<Impl>(concurrency))
{
}

Scheduler::~Scheduler()
{
}

int Scheduler::add_workflow(
  WorkflowT workflow)
{
  impl_->stop_finished_worker();
  int worker_id = impl_->get_available_worker();
  if (worker_id == -1) {
    impl_->add_queue(workflow);
    return -1;
  } else {
    impl_->start_worker(worker_id, workflow);
    return worker_id;
  }
}

void Scheduler::wait_till_all_complete() const
{
  for (auto & worker : impl_->workers) {
    worker.execution_future.wait();
  }
}

}  // namespace grasp_execution
