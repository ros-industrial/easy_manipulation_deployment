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

#ifndef EMD__GRASP_EXECUTION__CORE__SCHEDULER_HPP_
#define EMD__GRASP_EXECUTION__CORE__SCHEDULER_HPP_

#include <memory>
#include <future>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace grasp_execution
{

namespace core
{

using result_t = bool;

namespace Workflow
{

enum class Status : uint8_t
{
  COMPLETED = 1,
  QUEUED = 2,
  ONGOING = 3,
  CANCELLED = 4,
  INVALID = 5
};

}  // namespace Workflow

class Scheduler
{
public:
  typedef std::function<result_t(const std::string &)> WorkflowT;

  explicit Scheduler(size_t concurrency);

  ~Scheduler();

  Workflow::Status add_workflow(
    const std::string & workflow_id,
    WorkflowT workflow);

  Workflow::Status cancel_workflow(
    const std::string & workflow_id);

  void wait_till_all_complete() const;

  Workflow::Status wait_till_complete(
    const std::string & workflow_id,
    result_t & result) const;

  Workflow::Status get_status(
    const std::string & workflow_id) const;

  const result_t & get_result(const std::string & workflow_id) const;

  class Impl;

private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace core

}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__CORE__SCHEDULER_HPP_
