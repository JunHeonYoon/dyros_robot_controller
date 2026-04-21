// Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
//
// Developed by Yoon Junheon at the Dynamic Robotic Systems Laboratory (DYROS),
// Seoul National University, under a research agreement with ETRI.
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

#ifndef SUHAN_BENCHMARK_H
#define SUHAN_BENCHMARK_H


#include <iostream>
#include <fstream>
#include <chrono>

/**
 * @brief Lightweight wall-clock timer utility.
 */
class SuhanBenchmark
{
public:
  /**
   * @brief Constructor. Starts timing immediately.
   */
  SuhanBenchmark() : beg_(hd_clock::now()) {}
  /**
   * @brief Reset the timer start point to now.
   */
  void reset() { beg_ = hd_clock::now(); }
  /**
   * @brief Get elapsed time since last reset/creation.
   * @return (double) Elapsed time in seconds.
   */
  double elapsed() const 
  { 
    return std::chrono::duration_cast<second>(hd_clock::now() - beg_).count(); 
  }
  /**
   * @brief Get elapsed time and reset timer in one call.
   * @return (double) Elapsed time in seconds before reset.
   */
  double elapsedAndReset() 
  { 
    double e = elapsed(); 
    reset();
    return e;
  }

private:
  typedef std::chrono::high_resolution_clock hd_clock;
  typedef std::chrono::duration<double, std::ratio<1> > second;
  std::chrono::time_point<hd_clock> beg_;
};

#endif
