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

#include "emd/interpolate.hpp"
#include "rclcpp/time.hpp"

namespace emd
{

namespace core
{

void interpolate_between_points(
  rclcpp::Duration time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
  rclcpp::Duration time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
  rclcpp::Duration sample_time,
  trajectory_msgs::msg::JointTrajectoryPoint & output)
{
  interpolate_between_points(
    time_a.seconds(), state_a,
    time_b.seconds(), state_b,
    sample_time.seconds(), output);
}

void interpolate_between_points(
  double time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
  double time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
  double sample_time,
  trajectory_msgs::msg::JointTrajectoryPoint & output)
{
  double duration_so_far = sample_time - time_a;
  double duration_btwn_points = time_b - time_a;

  const size_t dim = state_a.positions.size();
  output.positions.resize(dim, 0.0);
  output.velocities.resize(dim, 0.0);
  output.accelerations.resize(dim, 0.0);

  // Generate size n, base x power series.
  // e.g. n = 5, x = 2
  // {1, 2, 4, 8, 16, 32, 64}
  auto generate_powers = [](int n, double x, double * powers)
    {
      powers[0] = 1.0;
      for (int i = 1; i <= n; ++i) {
        powers[i] = powers[i - 1] * x;
      }
    };

  bool has_velocity = !state_a.velocities.empty() && !state_b.velocities.empty();
  bool has_accel = !state_a.accelerations.empty() && !state_b.accelerations.empty();

  if (duration_so_far < 0.0) {
    duration_so_far = duration_btwn_points;
    has_velocity = has_accel = false;
  }

  double t[6];
  generate_powers(5, duration_so_far, t);

  if (!has_velocity && !has_accel) {
    // do linear interpolation
    double coefficients[2] = {0.0, 0.0};
    for (size_t i = 0; i < dim; ++i) {
      double start_pos = state_a.positions[i];
      double end_pos = state_b.positions[i];

      coefficients[0] = start_pos;
      if (duration_btwn_points > 0.0) {
        coefficients[1] = (end_pos - start_pos);
      }

      output.positions[i] = t[0] * coefficients[0] +
        t[1] * coefficients[1];
      output.velocities[i] = t[0] * coefficients[1];
    }
  } else if (has_velocity && !has_accel) {
    // do cubic interpolation
    double T[4];
    generate_powers(3, duration_btwn_points, T);

    for (size_t i = 0; i < dim; ++i) {
      double start_pos = state_a.positions[i];
      double start_vel = state_a.velocities[i];
      double end_pos = state_b.positions[i];
      double end_vel = state_b.velocities[i];

      double coefficients[4] = {0.0, 0.0, 0.0, 0.0};
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      if (duration_btwn_points > 0.0) {
        coefficients[2] =
          (-3.0 * start_pos + 3.0 * end_pos - 2.0 * start_vel * T[1] - end_vel * T[1]) / T[2];
        coefficients[3] =
          ( 2.0 * start_pos - 2.0 * end_pos + start_vel * T[1] + end_vel * T[1]) / T[3];
      }

      output.positions[i] = t[0] * coefficients[0] +
        t[1] * coefficients[1] +
        t[2] * coefficients[2] +
        t[3] * coefficients[3];
      output.velocities[i] = t[0] * coefficients[1] +
        t[1] * 2.0 * coefficients[2] +
        t[2] * 3.0 * coefficients[3];
      output.accelerations[i] = t[0] * 2.0 * coefficients[2] +
        t[1] * 6.0 * coefficients[3];
    }
  } else if (has_velocity && has_accel) {
    // do quintic interpolation
    double T[6];
    generate_powers(5, duration_btwn_points, T);

    for (size_t i = 0; i < dim; ++i) {
      double start_pos = state_a.positions[i];
      double start_vel = state_a.velocities[i];
      double start_acc = state_a.accelerations[i];
      double end_pos = state_b.positions[i];
      double end_vel = state_b.velocities[i];
      double end_acc = state_b.accelerations[i];

      double coefficients[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      coefficients[2] = 0.5 * start_acc;
      if (duration_btwn_points != 0.0) {
        coefficients[3] =
          (-20.0 * start_pos + 20.0 * end_pos - 3.0 * start_acc * T[2] + end_acc * T[2] -
          12.0 * start_vel * T[1] - 8.0 * end_vel * T[1]) / (2.0 * T[3]);
        coefficients[4] =
          (30.0 * start_pos - 30.0 * end_pos + 3.0 * start_acc * T[2] - 2.0 * end_acc * T[2] +
          16.0 * start_vel * T[1] + 14.0 * end_vel * T[1]) / (2.0 * T[4]);
        coefficients[5] = (-12.0 * start_pos + 12.0 * end_pos - start_acc * T[2] + end_acc * T[2] -
          6.0 * start_vel * T[1] - 6.0 * end_vel * T[1]) / (2.0 * T[5]);
      }

      output.positions[i] = t[0] * coefficients[0] +
        t[1] * coefficients[1] +
        t[2] * coefficients[2] +
        t[3] * coefficients[3] +
        t[4] * coefficients[4] +
        t[5] * coefficients[5];
      output.velocities[i] = t[0] * coefficients[1] +
        t[1] * 2.0 * coefficients[2] +
        t[2] * 3.0 * coefficients[3] +
        t[3] * 4.0 * coefficients[4] +
        t[4] * 5.0 * coefficients[5];
      output.accelerations[i] = t[0] * 2.0 * coefficients[2] +
        t[1] * 6.0 * coefficients[3] +
        t[2] * 12.0 * coefficients[4] +
        t[3] * 20.0 * coefficients[5];
    }
  }
}

}  // namespace core

}  // namespace emd
