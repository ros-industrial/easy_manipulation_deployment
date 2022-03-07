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

#ifndef EMD__DYNAMIC_SAFETY__SAFETY_ZONE_HPP_
#define EMD__DYNAMIC_SAFETY__SAFETY_ZONE_HPP_

#include <algorithm>
#include <cstdint>
#include <string>
#include <stdexcept>


namespace dynamic_safety
{

/// Safety Zone for dynamic safety.
class SafetyZone
{
public:
  /// Safety Zone option
  struct Option
  {
    /// Whether option is loaded by the overall dynamic safety module.
    bool manual;

    // TODO(Briancbn): Enable distance based.
    /// Unit type used.
    std::string unit_type;

    /// The time it takes to finish collision checking.
    double collision_checking_deadline;

    /// The time it takes for the robot to slow down.
    double slow_down_time;

    /// The time it takes for replanning.
    double replan_deadline;

    /// The time to look ahead.
    double look_ahead_time;
  };

  /// Constructor.
  SafetyZone()
  {
  }

  /// Constructor with option.
  /**
   * Safety zone initiated with option.
   * This will implicitly call set().
   *
   * \sa set()
   * \param[in] option Safety zone option.
   * \throw Error if the option timing is invalid.
   */
  explicit SafetyZone(const Option & option)
  {
    if (!set(option)) {
      throw std::runtime_error("Invalid option!");
    }
  }

  /// Flags for the 5 zones.
  static const uint8_t
    BLIND = 0,
    EMERGENCY = 1,
    SLOWDOWN = 2,
    REPLAN = 3,
    SAFE = 4;

  /// Zone limit: Last zone is not needed since it's infinite.
  /*
   * ```
   * zone:
   *    -------      0
   *    BLIND
   *    -------      value1
   *    EMERGENCY
   *    -------      value2
   *    SLOWDOWN
   *    -------      value3
   *    REPLAN
   *    -------      value4
   *    SAFE
   * ```
   */
  bool verify();

  /// Update safety zone option.
  /**
   * Based on the timing specified in option,
   * safety zone limit will be calculated.
   * If option is valid, the following output will be printed
   * in the terminal
   * ```
   * zone:
   *    -------      0
   *    BLIND
   *    -------      value1
   *    EMERGENCY
   *    -------      value2
   *    SLOWDOWN
   *    -------      value3
   *    REPLAN
   *    -------      value4
   *    SAFE
   * ```
   *
   * if slow down time is unset, it will be determined dynamically
   * and value2 would replaced with
   *
   * \param[in] option Safety zone option.
   */
  bool set(const Option & option);

  /// Get the zone the time point resides
  /**
   * \param[in] point Time point from start.
   * \return The safety zone it belongs.
   */
  uint8_t get_zone(double point) const;

  /// Get zone limit.
  /**
   * As there is no limit for SAFE, Segmentation fault will occur.
   *
   * \param[in] zone The name of the zone.
   */
  const double & get_zone_limit(uint8_t zone) const
  {
    return zone_[zone];
  }

  /// Get non const zone limit.
  /**
   * As there is no limit for SAFE, Segmentation fault will occur.
   *
   * \param[in] zone The name of the zone.
   */
  double & get_zone_limit_non_const(uint8_t zone)
  {
    return zone_[zone];
  }

  /// Get the unit type
  /**
   * Right now only seconds is allowed.
   *
   * \todo Other time unit or distanced based unit.
   *
   * \return The type of unit that the safety zone used.
   */
  const std::string & get_unit_type() const
  {
    return unit_type_;
  }

  void print() const;

private:
  // Safety zone unit type
  std::string unit_type_;

  // Zone limit
  double zone_[4];
};

}  // namespace dynamic_safety


#endif  // EMD__DYNAMIC_SAFETY__SAFETY_ZONE_HPP_
