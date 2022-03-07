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

#include "emd/dynamic_safety/safety_zone.hpp"


namespace dynamic_safety
{

bool SafetyZone::verify()
{
  // check if vector increasing
  for (int i = 0; i < 3; i++) {
    if (zone_[i] > zone_[i + 1]) {
      return false;
    }
  }
  return true;
}

bool SafetyZone::set(const Option & option)
{
  unit_type_ = option.unit_type;
  zone_[BLIND] = option.collision_checking_deadline;

  // Check if slow down zone is defined
  // as this one could be defined dynamically.
  if (option.slow_down_time > 0) {
    zone_[EMERGENCY] =
      zone_[BLIND] + option.slow_down_time;
  } else {
    zone_[EMERGENCY] = zone_[BLIND];
  }

  zone_[SLOWDOWN] = zone_[EMERGENCY] + option.replan_deadline;

  // Check if look_ahead_time is long enough
  if (option.look_ahead_time >= zone_[SLOWDOWN]) {
    zone_[REPLAN] = option.look_ahead_time;
  } else {
    fprintf(
      stderr, "Look ahead time [%fs] is too short [<%fs]\n",
      option.look_ahead_time, zone_[SLOWDOWN]);
    fflush(stderr);
    return false;
  }
  if (verify()) {
    return true;
  } else {
    fprintf(stderr, "Zone limit is not increasing");
    fflush(stderr);
    return false;
  }
}

uint8_t SafetyZone::get_zone(double point) const
{
  for (size_t i = 0; i < 4; i++) {
    if (point < zone_[i]) {
      return static_cast<uint8_t>(i);
    }
  }
  return SAFE;
}

void SafetyZone::print() const
{
  printf(
    "Safety Zone:\n"
    "   -------      0\n"
    "   BLIND\n"
    "   -------      %.2e\n"
    "   EMERGENCY\n"
    "   -------      %.2e\n"
    "   SLOWDOWN\n"
    "   -------      %.2e\n"
    "   REPLAN\n"
    "   -------      %.2e\n"
    "   SAFE\n",
    zone_[BLIND], zone_[EMERGENCY], zone_[SLOWDOWN], zone_[REPLAN]);
  fflush(stdout);
}

}  // namespace dynamic_safety
