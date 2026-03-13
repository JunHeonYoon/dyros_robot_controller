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
