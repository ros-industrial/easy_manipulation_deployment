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

#ifndef EMD__PROFILER_HPP_
#define EMD__PROFILER_HPP_

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>


namespace emd
{

/// A profiler base that record data.
template<typename T>
class Profiler
{
public:
  /// Create a new profiler with fixed recording buffer size.
  /**
   * Setting a data size before starting the profiler help mitigate
   * dynamic memory allocation involved when changing buffer size.
   *
   * \param[in] size Buffer size for recording statistics.
   */
  explicit Profiler(size_t size)
  {
    database_.resize(size);
    itr_ = 0;
  }

  /// Record data into the profiler.
  /**
   * This function is thread safe.
   *
   * \param[in] data Data to record.
   * \return The reference idiom to the stored data.
   */
  const T & record(T data)
  {
    size_t itr = itr_++;
    if (itr < database_.size()) {
      database_[itr] = data;
    }
    return database_[itr];
  }

  /// Get recorded data.
  /**
   * This function will automatically remove buffer that
   * hasn't been filled by a recorded data.
   *
   * \return A copy of all the recorded data.
   */
  std::vector<T> get() const
  {
    std::vector<T> result;
    int itr = static_cast<int>(itr_);
    std::copy(
      database_.begin(), database_.begin() + itr,
      std::back_inserter(result));
    return result;
  }

  /// Destructor.
  ~Profiler()
  {
  }

private:
  std::vector<T> database_;
  std::atomic_ulong itr_;
};


/// Profiler that act like a stopwatch.
/// By default everything is seconds in double format.
template<typename Duration = std::chrono::duration<double, std::ratio<1>>>
class TimeProfiler : public Profiler<Duration>
{
public:
  /// Create a new time profiler with fixed recording buffer size.
  /**
   * Setting a data size before starting the profiler help mitigate
   * dynamic memory allocation involved when changing buffer size.
   *
   * \param[in] size Buffer size for recording statistics.
   */
  explicit TimeProfiler(size_t size)
  : Profiler<Duration>(size)
  {
  }

  /// Start the stopwatch.
  /**
   * Equivalent to reset().
   *
   * \sa reset()
   */
  void start()
  {
    start_time = std::chrono::steady_clock::now();
  }

  /// Lapse and record the time taken.
  /**
   * \return time lapse between now and the start time.
   */
  double lapse_and_record()
  {
    return static_cast<double>(this->record(
             std::chrono::steady_clock::now() - start_time).count());
  }

  /// Lapse but don't record the time taken.
  /**
   * \return time lapse between now and the start time.
   */
  double lapse()
  {
    return static_cast<double>((
             std::chrono::steady_clock::now() - start_time).count());
  }

  /// Reset the stopwatch.
  /**
   * Equivalent to start().
   *
   * \sa start()
   */
  void reset()
  {
    start();
  }

  /// Print the recorded result
  /**
   * \param[in] os Output stream.
   * \param[in] metadata_only Whether to print out full data or just meta data.
   */
  void print(std::ostream & os = std::cout, bool metadata_only = false)
  {
    const auto & result_database = this->get();
    double average = 0;
    double sq_average = 0;
    double temp;
    for (int i = 0; i < static_cast<int>(result_database.size()); i++) {
      temp = result_database[static_cast<size_t>(i)].count();
      if (!metadata_only) {
        os << "Data " << i + 1 << ": " << temp << std::endl;
      }
      average = (average * i + temp) / (i + 1);
      sq_average = (sq_average * i + pow(temp, 2.0)) / (i + 1);
    }
    os << "Average: " << average << std::endl;
    os << "Standard Deviation: " << sqrt(sq_average - pow(average, 2.0)) << std::endl;
  }

private:
  std::chrono::time_point<std::chrono::steady_clock> start_time;
};

}  // namespace emd


#endif  // EMD__PROFILER_HPP_
