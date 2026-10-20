// Copyright 2024-2026 CARI-JRL (CNR-STIIMA / University of Brescia), SUPSI, Politecnico di Milano and contributors
// SPDX-License-Identifier: Apache-2.0

/**
 * @file microinterpolator.h
 * @brief Polynomial interpolation of a joint trajectory between its waypoints.
 */

#pragma once
#include <string>
#include <vector>

#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

/**
 * @brief Interpolates a `trajectory_msgs/JointTrajectory` at arbitrary times.
 *
 * Between two consecutive waypoints each joint follows a polynomial that matches the waypoint
 * positions and, depending on the order (see setSplineOrder()), velocities and accelerations.
 *
 * Typical use in a controller:
 * @code
 * auto interp = std::make_shared<Microinterpolator>();   // outside the real-time loop
 * interp->setSplineOrder(1);
 * interp->setTrajectory(trajectory);                      // O(number of points)
 * // ... swap the pointer used by the real-time loop under a short lock ...
 * interp->interpolate(scaled_time, point, override);      // real-time safe
 * @endcode
 *
 * The class is not thread safe: setTrajectory() and interpolate() must not run concurrently on the
 * same instance.
 */
class Microinterpolator
{
protected:

  trajectory_msgs::msg::JointTrajectory trj_;  ///< Trajectory being interpolated.
  std::vector<double> times_;       ///< time_from_start of each point, in seconds.
  size_t segment_{1};               ///< Segment used by the last interpolate() call.
  const char* last_error_{nullptr}; ///< Reason of the last interpolate() failure.

  unsigned int order_;              ///< Spline order, see setSplineOrder().
  bool trj_set_;                    ///< True once a trajectory has been set.

  /// @brief Caches the waypoint times and resets the segment search.
  bool init();
public:
  Microinterpolator();

  /**
   * @brief Sets the trajectory to interpolate.
   *
   * O(number of points): call it outside the real-time loop.
   * Waypoint times must be strictly increasing (as checked by the joint trajectory controller).
   * Orders >= 1 also need the `velocities` of every point, orders >= 2 the `accelerations`.
   *
   * @param trj trajectory, with the joints already in the controller order
   * @return false if the trajectory has no points
   */
  bool setTrajectory(const trajectory_msgs::msg::JointTrajectory& trj);
  /// @copydoc setTrajectory(const trajectory_msgs::msg::JointTrajectory&)
  bool setTrajectory(trajectory_msgs::msg::JointTrajectory&& trj);

  /// @brief Returns a copy of the current trajectory.
  trajectory_msgs::msg::JointTrajectory getTrajectory(){return trj_;}

  /**
   * @brief Sets the order of the interpolating polynomial.
   *
   * - order = 0: positions are continuous (linear interpolation)
   * - order = 1: velocities are continuous (cubic)
   * - order = 2: accelerations are continuous (quintic)
   * - order = 3: jerks are continuous, supposed zero at the waypoints (7th degree)
   * - order = 4: snaps are continuous, supposed zero at the waypoints (9th degree)
   *
   * Values greater than 4 are rejected with a warning and the order is left unchanged.
   */
  void setSplineOrder(const unsigned int& order);

  /**
   * @brief Computes the trajectory point at a given time.
   *
   * Real-time safe: no logging, and no allocation once @p pnt has the right size. The segment
   * search starts from the segment used by the previous call, so advancing time costs O(1).
   *
   * - Before the first waypoint: returns the first waypoint and false.
   * - After the last waypoint: returns the last waypoint with zero velocity/acceleration and true.
   *
   * @param time    time from the start of the trajectory
   * @param pnt     output point (positions, velocities, accelerations, effort set to zero)
   * @param scaling speed override: velocities are multiplied by it, accelerations by its square
   * @return false on failure; lastError() tells why
   */
  bool interpolate(const rclcpp::Duration& time, trajectory_msgs::msg::JointTrajectoryPoint& pnt, const double& scaling=1.0);

  /// @brief Reason of the last interpolate() failure (empty string if none).
  const char* lastError() const {return last_error_ ? last_error_ : "";}

  /// @brief Duration of the trajectory (time_from_start of the last point), zero if not set.
  rclcpp::Duration trjTime();

  /**
   * @brief Human readable description: number of points, duration, first and last points.
   * @param n_head number of initial points to print
   * @param n_tail number of final points to print
   */
  std::string summary(size_t n_head=3, size_t n_tail=3) const;
};
