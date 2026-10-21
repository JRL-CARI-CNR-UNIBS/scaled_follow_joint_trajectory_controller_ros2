// Copyright 2024-2026 CARI-JRL (CNR-STIIMA / University of Brescia), SUPSI, Politecnico di Milano and contributors
// SPDX-License-Identifier: Apache-2.0

/**
 * @file scaled_fjt_controller.hpp
 * @brief FollowJointTrajectory controller whose execution speed can be scaled at runtime.
 */

#ifndef SCALED_FJT_CONTROLLER
#define SCALED_FJT_CONTROLLER

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "scaled_fjt_controller/async_logger.hpp"
#include "scaled_fjt_controller/microinterpolator.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"

/// @brief Scaled FollowJointTrajectory controller.
namespace scaled_fjt_controller
{

/**
 * @brief Joint trajectory controller with runtime speed scaling.
 *
 * The controller exposes a `control_msgs/action/FollowJointTrajectory` action server, like
 * `joint_trajectory_controller/JointTrajectoryController` (from which it inherits the
 * parameters and the command/state interfaces), and follows the received trajectory along a
 * *scaled time*:
 *
 * @f[ \tau_{k+1} = \tau_k + \Delta t \cdot s_k @f]
 *
 * where @f$\Delta t@f$ is the control period and @f$s_k \in [0, 1]@f$ is the product of the
 * speed overrides received on the `speed_ovr_topics`. With @f$s=1@f$ the trajectory is executed at
 * nominal speed, with @f$s=0@f$ the robot stops on the path and resumes when the override grows.
 * Between waypoints the trajectory is interpolated by Microinterpolator with a polynomial whose
 * order is set by the `spline_order` parameter.
 *
 * @par Real-time design
 * - update() never blocks on work proportional to the trajectory length: goal callbacks prepare
 *   the new Microinterpolator outside the lock and only swap a pointer under it.
 * - update() never logs directly (see AsyncLogger), and publishes through
 *   `realtime_tools::RealtimePublisher` (try-lock, the message is dropped if the publisher is busy).
 * - update() never touches the action goal handle: it only flags the goal as finished, and a
 *   non-real-time timer (period `1 / action_monitor_rate`) sends the action result.
 * - ROS entities (subscriptions, publishers, action server, timer) are created in on_configure(),
 *   because on_activate() is executed by the controller manager inside the real-time loop.
 *
 * See `docs/ros_api.md` for the full list of parameters, topics and action behaviour.
 */
class ScaledFjtController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  ScaledFjtController() = default;
  ~ScaledFjtController() override = default;

  /// @brief Creates the asynchronous logger, then calls the base class on_init().
  CallbackReturn on_init() override;

  /**
   * @brief Configures the base class, then reads `speed_ovr_topics` and `spline_order` and creates
   *        the override subscriptions, the action server, the publishers and the goal monitor timer.
   */
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /// @brief Same state interfaces as the base class (position and, optionally, velocity).
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Activates the base class and starts holding the current joint position.
   *
   * Runs in the real-time loop: it only reads the state interfaces and builds a one-point trajectory.
   */
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Real-time loop: interpolates the trajectory at the scaled time and writes the commands.
   *
   * @param time   current time, used to stamp the published messages
   * @param period time since the last update; the scaled time advances by `period * override`
   * @return always `controller_interface::return_type::OK`
   */
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /// @brief Validates a new goal (delegates to the base class).
  rclcpp_action::GoalResponse goal_received_callback(const rclcpp_action::GoalUUID& uuid,
                                                     std::shared_ptr<const FollowJTrajAction::Goal> goal);

  /// @brief Stops the robot at the last commanded position and cancels the goal.
  rclcpp_action::CancelResponse goal_cancelled_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  /**
   * @brief Starts executing a new goal (preempting the current one).
   *
   * The trajectory is reordered to the controller joint order and loaded into a new
   * Microinterpolator outside the real-time lock; only the pointer swap happens under it.
   * Goals whose joint names do not match the controller joints are aborted with `INVALID_JOINTS`.
   */
  void goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  /**
   * @brief Callback of the speed override topics.
   *
   * @param msg   override in percent; values are clamped to [0, 100]
   * @param topic topic the message comes from; the global override is the product of the last
   *              value received on each topic
   */
  void SpeedOvrCb(const std_msgs::msg::Int16& msg, const std::string& topic);

  std::shared_ptr<Microinterpolator> microinterpolator_;   ///< Interpolator used by update().
  trajectory_msgs::msg::JointTrajectoryPoint current_point_;  ///< Last commanded point.

  std::atomic<double> speed_ovr_{1.0};                     ///< Global speed override in [0, 1].
  std::map<std::string, double> speed_ovr_map_;            ///< Last override received on each topic, in [0, 1].
  std::vector<rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr> speed_ovr_sub_;  ///< Override subscriptions.

protected:
  /// @brief Nominal and scaled time elapsed since the start of the current trajectory.
  struct TimeData
  {
    TimeData() : time(rclcpp::Duration::from_seconds(0.0)), scaled_time(rclcpp::Duration::from_seconds(0.0)) {}
    rclcpp::Duration time;         ///< Wall-clock time since the trajectory started.
    rclcpp::Duration scaled_time;  ///< Time along the trajectory (advances by `period * override`).
  };

  using Float64Publisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64>;
  using JointStatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>;

  /**
   * @brief Reorders the joints of @p trj following @p joint_names.
   * @return false if the two sets of joint names differ
   */
  bool sort_trajectory(const std::vector<std::string>& joint_names, const trajectory_msgs::msg::JointTrajectory& trj,
                       trajectory_msgs::msg::JointTrajectory& sorted_trj);

  /// @brief Non-real-time timer callback: sends the result of the goal that update() flagged as finished.
  void goalMonitorCallback();

  /// @brief Logs, about once per second, the worst update() duration and period measured by update().
  void reportTiming();

  ScaledFjtController::TimeData td_;  ///< Trajectory time. Protected by #mtx_.

  /// Shared with update(). Goal callbacks hold it only to swap pointers (a few microseconds).
  std::mutex mtx_;
  /// Serializes the non-real-time goal handling (callbacks may run in parallel executor threads).
  std::mutex goal_mtx_;
  /// Protects #speed_ovr_map_.
  std::mutex speed_ovr_mtx_;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle_;  ///< Executing goal. Protected by #goal_mtx_.
  uint64_t goal_id_{0};             ///< Id of #goal_handle_. Protected by #goal_mtx_.
  uint64_t goal_counter_{0};        ///< Last id assigned to a goal. Protected by #goal_mtx_.
  uint64_t active_goal_id_{0};      ///< Goal executed by update(), 0 if none. Protected by #mtx_.
  std::atomic<uint64_t> finished_goal_id_{0};  ///< Last goal completed by update(), read by goalMonitorCallback().
  rclcpp::TimerBase::SharedPtr goal_monitor_timer_;  ///< Runs goalMonitorCallback().

  unsigned int spline_order_{1};    ///< Order passed to Microinterpolator::setSplineOrder().
  std::unique_ptr<AsyncLogger> async_log_;  ///< Logging from update() and from the goal callbacks.
  trajectory_msgs::msg::JointTrajectoryPoint unscaled_pnt_;  ///< Reused by update() to avoid allocations.
  bool debug_enabled_{false};       ///< Cached "DEBUG level enabled", refreshed about once per second.
  unsigned int debug_check_counter_{0};
  std::chrono::steady_clock::time_point last_interpolation_error_{};  ///< Throttles interpolation errors.

  // update() timing. Written by update(), read and reset by reportTiming().
  std::chrono::steady_clock::time_point last_update_start_{};  ///< Start of the previous update(), zero if none.
  std::atomic<int64_t> max_update_ns_{0};   ///< Longest update() since the last report, lock wait included.
  std::atomic<int64_t> max_period_ns_{0};   ///< Longest time between two update() starts since the last report.
  std::atomic<uint32_t> late_cycles_{0};    ///< Periods longer than #late_threshold_ns_ since the last report.
  std::atomic<int64_t> validation_ns_{0};   ///< Duration of the last goal_received_callback().
  int64_t late_threshold_ns_{0};            ///< 1.5 nominal periods, 0 if the update rate is unknown.
  std::chrono::steady_clock::time_point last_timing_report_{};  ///< Used by reportTiming().
  std::chrono::steady_clock::time_point last_goal_time_{};      ///< Last accepted goal. Protected by #goal_mtx_.

  std::unique_ptr<Float64Publisher> scaled_time_pub_;        ///< Topic `scaled_time`.
  std::unique_ptr<Float64Publisher> execution_ratio_pub_;    ///< Topic `execution_ratio`.
  std::unique_ptr<JointStatePublisher> unscaled_joint_target_pub_;  ///< Topic `unscaled_joint_target`.

private:
  std::vector<std::string> joint_names_;  ///< Controller joint names, in command interface order.

  /// @brief Scaled time and current command, as text (for log messages).
  std::string printCurrentPos();
};

}  // namespace scaled_fjt_controller

#endif
