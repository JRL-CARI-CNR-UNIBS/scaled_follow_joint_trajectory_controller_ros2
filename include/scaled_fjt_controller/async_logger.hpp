// Copyright 2024-2026 CARI-JRL (CNR-STIIMA / University of Brescia), SUPSI, Politecnico di Milano and contributors
// SPDX-License-Identifier: Apache-2.0

/**
 * @file async_logger.hpp
 * @brief Non-blocking logging for real-time code.
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rcutils/logging.h>

namespace scaled_fjt_controller
{

/**
 * @brief Logs messages from a background thread, so that callers (e.g. the real-time update loop)
 * never wait on stdout, /rosout or a contended mutex.
 *
 * push() only does a try_lock: if the queue is busy or full, the message is dropped and counted.
 * The number of dropped messages is reported by the worker thread.
 */
class AsyncLogger
{
public:
  /// @brief Severity of a message.
  enum class Level { Debug, Info, Warn, Error };

  /**
   * @brief Starts the worker thread.
   * @param logger    logger used by the worker thread
   * @param max_queue messages kept in the queue before new ones are dropped
   */
  explicit AsyncLogger(const rclcpp::Logger& logger, size_t max_queue = 200)
    : logger_(logger), max_queue_(max_queue), worker_([this] { run(); })
  {
  }

  /// @brief Writes the queued messages and stops the worker thread.
  ~AsyncLogger()
  {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      stop_ = true;
    }
    cv_.notify_one();
    worker_.join();
  }

  AsyncLogger(const AsyncLogger&) = delete;
  AsyncLogger& operator=(const AsyncLogger&) = delete;

  /// @brief True if the logger prints DEBUG messages. Use it to avoid building strings that would be discarded.
  bool debugEnabled() const
  {
    return rcutils_logging_logger_is_enabled_for(logger_.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  }

  /**
   * @brief Queues a message for the worker thread. Never blocks.
   * @return false if the message was dropped (queue busy or full)
   */
  bool push(Level level, std::string msg)
  {
    std::unique_lock<std::mutex> lock(mtx_, std::try_to_lock);
    if (!lock.owns_lock() || queue_.size() >= max_queue_)
    {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    queue_.emplace_back(level, std::move(msg));
    lock.unlock();
    cv_.notify_one();
    return true;
  }

private:
  void run()
  {
    std::deque<std::pair<Level, std::string>> batch;
    while (true)
    {
      {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
        if (stop_ && queue_.empty())
          return;
        batch.swap(queue_);
      }
      for (const auto& [level, msg] : batch)
        write(level, msg);
      batch.clear();

      const size_t dropped = dropped_.exchange(0, std::memory_order_relaxed);
      if (dropped > 0)
        RCLCPP_WARN(logger_, "AsyncLogger: %zu messages dropped", dropped);
    }
  }

  void write(Level level, const std::string& msg) const
  {
    switch (level)
    {
      case Level::Debug: RCLCPP_DEBUG(logger_, "%s", msg.c_str()); break;
      case Level::Info:  RCLCPP_INFO(logger_, "%s", msg.c_str()); break;
      case Level::Warn:  RCLCPP_WARN(logger_, "%s", msg.c_str()); break;
      case Level::Error: RCLCPP_ERROR(logger_, "%s", msg.c_str()); break;
    }
  }

  rclcpp::Logger logger_;
  size_t max_queue_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<std::pair<Level, std::string>> queue_;
  std::atomic<size_t> dropped_{0};
  bool stop_{false};
  std::thread worker_;  // last member: started after everything else is initialized
};

}  // namespace scaled_fjt_controller
