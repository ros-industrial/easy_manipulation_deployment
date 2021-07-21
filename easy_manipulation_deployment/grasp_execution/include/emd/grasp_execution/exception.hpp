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

#ifndef EMD__GRASP_EXECUTION__EXCEPTION_HPP_
#define EMD__GRASP_EXECUTION__EXCEPTION_HPP_

#include <stdexcept>
#include <string>

namespace grasp_execution
{

/// Exception for job failure within a workflow
struct JobException : public std::runtime_error
{
  /// Constructor
  /**
   * \param[in] job_name the name of the failed job
   * \param[in] error_message additional error message
   */
  JobException(
    const std::string & job_name,
    const std::string & error_message = "")
  : std::runtime_error("[" + job_name + "] failed!" +
      (error_message.empty() ? "" : "\nMessage: " + error_message))
  {
  }
};


/// Context loading exception.
struct ContextLoadingException : public std::runtime_error
{
  /// Constructor
  /**
   * \param[in] missing_field the name of the missing compulsory field.
   * \param[in] parent_field the parent name of the missing compulsory field.
   */
  ContextLoadingException(
    const std::string & missing_field,
    const std::string & parent_field = "")
  : std::runtime_error("could not load workcell context, config file is invalid, " +
      (parent_field.empty() ? "" : "[" + parent_field + "] ") +
      "missing field [" + missing_field + "]")
  {
  }
};


}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__EXCEPTION_HPP_
