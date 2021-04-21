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

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "grasp_execution/core/scheduler.hpp"

TEST(TestScheduler, SingleThreaded) {
  grasp_execution::core::Scheduler sch(1);
  using sch_status = grasp_execution::core::Workflow::Status;
  std::vector<std::string> flags;
  auto test_workflow = [&flags](const std::string & id) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      flags.push_back(id);
      return id.size() > 2;
    };

  // Check if workflow 1 is started
  ASSERT_EQ(sch.add_workflow("w1", test_workflow), sch_status::ONGOING);

  // Check if repeated ongoing workflow is rejected
  EXPECT_EQ(sch.add_workflow("w1", test_workflow), sch_status::INVALID);

  // Check if workflow 2 is queued
  EXPECT_EQ(sch.add_workflow("w_2", test_workflow), sch_status::QUEUED);

  // Check if repeated queued workflow is rejected
  EXPECT_EQ(sch.add_workflow("w_2", test_workflow), sch_status::INVALID);

  // Check if workflow 3 is queued
  EXPECT_EQ(sch.add_workflow("w_3", test_workflow), sch_status::QUEUED);

  bool result;

  // Wait till ongoing workflow 1 complete
  EXPECT_EQ(sch.wait_till_complete("w1", result), sch_status::COMPLETED);

  // Confirm workflow 3 is still in queued status
  EXPECT_EQ(sch.get_status("w_3"), sch_status::QUEUED);

  // Wait till queued workflow 3 complete
  EXPECT_EQ(sch.wait_till_complete("w_3", result), sch_status::COMPLETED);

  EXPECT_TRUE(result);

  EXPECT_EQ(flags.size(), 3u);

  EXPECT_TRUE((flags == std::vector<std::string>{"w1", "w_2", "w_3"}));
}
