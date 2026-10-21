// Copyright 2024-2026 CARI-JRL (CNR-STIIMA / University of Brescia), SUPSI, Politecnico di Milano and contributors
// SPDX-License-Identifier: Apache-2.0

#include <scaled_fjt_controller/scaled_fjt_controller.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include <rclcpp/version.h>

namespace scaled_fjt_controller
{

namespace
{
using Clock = std::chrono::steady_clock;

int64_t elapsedNs(Clock::time_point from, Clock::time_point to)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(to - from).count();
}

// Lock-free "a = max(a, v)" (single writer in practice, but the reader resets it concurrently).
void storeMax(std::atomic<int64_t>& a, int64_t v)
{
  int64_t cur = a.load(std::memory_order_relaxed);
  while (v > cur && !a.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {}
}

std::string toMs(int64_t ns)
{
  return std::to_string(ns * 1.0e-6) + " ms";
}
}  // namespace

std::string ScaledFjtController::printCurrentPos()
{
  std::string p = "time: "+std::to_string(td_.scaled_time.seconds())+ " pos: ";
  for (size_t i=0; i<current_point_.positions.size();i++)
  {
    p = p+std::to_string(current_point_.positions[i])+" ";
  }
  return p;
}

bool ScaledFjtController::sort_trajectory(const std::vector<std::string>& joint_names, const trajectory_msgs::msg::JointTrajectory& trj, trajectory_msgs::msg::JointTrajectory& sorted_trj)
{
  const std::vector<std::string>& names=trj.joint_names;
  if (names.size()!=joint_names.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Joint names dimensions are different");
    return false;
  }
  std::vector<int> order_idx(joint_names.size());

  for (unsigned int iOrder=0;iOrder<joint_names.size();iOrder++)
  {
    RCLCPP_DEBUG(get_node()->get_logger(),"index %u, original trajectory %s, sorted trajectory %s",iOrder,names.at(iOrder).c_str(),joint_names.at(iOrder).c_str());
    if (names.at(iOrder).compare(joint_names.at(iOrder)))
    {
      for (unsigned int iNames=0;iNames<names.size();iNames++)
      {
        if (!joint_names.at(iOrder).compare(names.at(iNames)))
        {
          order_idx.at(iOrder)=iNames;
          RCLCPP_DEBUG(get_node()->get_logger(),"Joint %s (index %u) of original trajectory will be in position %u",names.at(iNames).c_str(),iOrder,iNames);
          break;
        }
        if (iNames==(names.size()-1))
        {
          RCLCPP_ERROR(get_node()->get_logger(),"Joint %s missing",joint_names.at(iOrder).c_str());
          return false;
        }
      }
    }
    else
    {
      order_idx.at(iOrder)=iOrder;
      RCLCPP_DEBUG(get_node()->get_logger(),"Joint %s (index %u) of original trajectory will be in position %u",names.at(iOrder).c_str(),iOrder,iOrder);
    }
  }

  sorted_trj.joint_names=joint_names;
  sorted_trj.header=trj.header;

  sorted_trj.points.clear();
  sorted_trj.points.reserve(trj.points.size());
  for (const trajectory_msgs::msg::JointTrajectoryPoint& pnt: trj.points)
  {
    sorted_trj.points.push_back(pnt);
    for (unsigned int iOrder=0;iOrder<joint_names.size();iOrder++)
    {
      sorted_trj.points.back().positions.at(iOrder)=pnt.positions.at(order_idx.at(iOrder));
      if (pnt.velocities.size()>0)
        sorted_trj.points.back().velocities.at(iOrder)=pnt.velocities.at(order_idx.at(iOrder));
      if (pnt.accelerations.size()>0)
        sorted_trj.points.back().accelerations.at(iOrder)=pnt.accelerations.at(order_idx.at(iOrder));
      if (pnt.effort.size()>0)
        sorted_trj.points.back().effort.at(iOrder)=pnt.effort.at(order_idx.at(iOrder));
    }
  }
  return true;
}

controller_interface::CallbackReturn ScaledFjtController::on_init()
{
  // Created here and not in on_activate(), which runs in the real-time loop.
  async_log_ = std::make_unique<AsyncLogger>(get_node()->get_logger());
  return JointTrajectoryController::on_init();
}

controller_interface::CallbackReturn ScaledFjtController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  auto ret = JointTrajectoryController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
    return ret;

  // Everything below creates ROS entities: done here because on_activate() runs in the real-time loop.
  std::vector<std::string> speed_ovr_topics;
  if (!get_node()->has_parameter("speed_ovr_topics"))
  {
    speed_ovr_topics.push_back("/speed_ovr");
    speed_ovr_topics.push_back("/safe_ovr");
  }
  else
    speed_ovr_topics = get_node()->get_parameter("speed_ovr_topics").as_string_array();

  int spline_order ;
  if (!get_node()->has_parameter("spline_order"))
    spline_order = 1;
  else
    spline_order = get_node()->get_parameter("spline_order").as_int();

  if(spline_order<1)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(),"Spline order cannot be less than 1, set equal to 1");
    spline_order = 1;
  }
  spline_order_ = spline_order;

  speed_ovr_sub_.clear();
  {
    std::lock_guard<std::mutex> lock(speed_ovr_mtx_);
    speed_ovr_map_.clear();
    for(const std::string& topic: speed_ovr_topics)
      speed_ovr_map_.insert(std::pair<std::string,double>(topic,1.0));
  }
  speed_ovr_ = 1.0;
  for(const std::string& topic: speed_ovr_topics)
  {
    auto cb=[this,topic](const std_msgs::msg::Int16 msg){return this->SpeedOvrCb(msg,topic);};
    speed_ovr_sub_.push_back(get_node()->create_subscription<std_msgs::msg::Int16>(topic,10,cb));
    RCLCPP_INFO_STREAM(this->get_node()->get_logger(),"Subscribing speed override topic: "<<topic);
  }

  // Replaces the action server created by the base class, so that goals go through this class.
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
        get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
        get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
        std::string(get_node()->get_name()) + "/follow_joint_trajectory",
        std::bind(&ScaledFjtController::goal_received_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ScaledFjtController::goal_cancelled_callback, this, std::placeholders::_1),
        std::bind(&ScaledFjtController::goal_accepted_callback, this, std::placeholders::_1));

  // Real-time safe publishers: update() only try-locks them, a separate thread does the publishing.
  scaled_time_pub_ = std::make_unique<Float64Publisher>(
        get_node()->create_publisher<std_msgs::msg::Float64>("scaled_time", 10));
  execution_ratio_pub_ = std::make_unique<Float64Publisher>(
        get_node()->create_publisher<std_msgs::msg::Float64>("execution_ratio", 10));
  unscaled_joint_target_pub_ = std::make_unique<JointStatePublisher>(
        get_node()->create_publisher<sensor_msgs::msg::JointState>("unscaled_joint_target", 10));

  // A cycle is "late" when the time since the previous one exceeds 1.5 nominal periods.
  const unsigned int rate = get_update_rate();
  late_threshold_ns_ = rate > 0 ? static_cast<int64_t>(1.5e9 / rate) : 0;
  if (rate == 0)
    RCLCPP_WARN(get_node()->get_logger(), "Update rate unknown: late cycles will not be counted");

  // Non-real-time timer that sends the action result when update() reports the end of the trajectory.
  goal_monitor_timer_ = get_node()->create_wall_timer(
        action_monitor_period_.to_chrono<std::chrono::nanoseconds>(), [this] { goalMonitorCallback(); });

  return ret;
}

controller_interface::InterfaceConfiguration ScaledFjtController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf = JointTrajectoryController::state_interface_configuration();

  return conf;
}

controller_interface::CallbackReturn ScaledFjtController::on_activate(const rclcpp_lifecycle::State& state)
{
  auto ret = JointTrajectoryController::on_activate(state);

  // Runs in the real-time loop: only read the current position and start holding it.
  current_point_.time_from_start = rclcpp::Duration::from_seconds(0.0);
  current_point_.positions.resize(this->dof_, 0);
  current_point_.velocities.resize(this->dof_, 0);
  current_point_.accelerations.resize(this->dof_, 0);
  current_point_.effort.resize(this->dof_, 0);
  joint_names_.resize(this->dof_,"");

  std::string delimiter = "/position";
  for (size_t i=0; i<current_point_.positions.size();i++)
  {
    double jpos = this->joint_state_interface_[0][i].get().get_value();
    current_point_.positions[i] = jpos;

    joint_names_.at(i) = this->joint_state_interface_[0][i].get().get_name();
    joint_names_.at(i) = joint_names_.at(i).substr(0, joint_names_[i].find(delimiter));
  }

  trajectory_msgs::msg::JointTrajectory hold_trj;
  hold_trj.joint_names = joint_names_;
  hold_trj.points.push_back(current_point_);

  auto hold = std::make_shared<Microinterpolator>();
  hold->setSplineOrder(spline_order_);
  hold->setTrajectory(std::move(hold_trj));
  last_update_start_ = Clock::time_point{};  // do not count the inactive time as a period
  {
    std::lock_guard<std::mutex> lock(mtx_);
    td_.scaled_time = rclcpp::Duration::from_seconds(0.0);
    td_.time        = rclcpp::Duration::from_seconds(0.0);
    microinterpolator_ = std::move(hold);
    active_goal_id_ = 0;
  }

  return ret;
}

controller_interface::return_type ScaledFjtController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // Real-time loop: no blocking logging and no blocking publishing here.
  // Timing is taken before the lock, so the measured duration includes the wait for it.
  const Clock::time_point begin = Clock::now();
  if (last_update_start_ != Clock::time_point{})
  {
    const int64_t period_ns = elapsedNs(last_update_start_, begin);
    storeMax(max_period_ns_, period_ns);
    if (late_threshold_ns_ > 0 && period_ns > late_threshold_ns_)
      late_cycles_.fetch_add(1, std::memory_order_relaxed);
  }
  last_update_start_ = begin;

  std::lock_guard<std::mutex> lock(mtx_); // goal callbacks hold it only to swap the interpolator

  const double speed_ovr = speed_ovr_.load();

  // Checking the logger level costs a little: do it about once per second.
  if (debug_check_counter_++ % 500 == 0)
    debug_enabled_ = async_log_->debugEnabled();

  if( !microinterpolator_->interpolate(td_.scaled_time,current_point_,speed_ovr) )
  {
    // Throttled to once per second, the message is built only when it is actually sent.
    if (begin - last_interpolation_error_ > std::chrono::seconds(1))
    {
      last_interpolation_error_ = begin;
      async_log_->push(AsyncLogger::Level::Error,
                       "something wrong in interpolation (" + std::string(microinterpolator_->lastError()) +
                       "): scaled time = " + std::to_string(td_.scaled_time.seconds()) +
                       ", global override = " + std::to_string(speed_ovr) + ", " + printCurrentPos());
    }
  }

  // End of the trajectory: only flag it, the action result is sent by goalMonitorCallback().
  if (active_goal_id_ != 0 && (td_.scaled_time-microinterpolator_->trjTime()).seconds()>=0)
  {
    finished_goal_id_.store(active_goal_id_);
    active_goal_id_ = 0;
  }

  // send command to robot
  if (has_position_command_interface_)
  {
    for (size_t i=0; i<current_point_.positions.size();i++)
      (void)this->joint_command_interface_[0][i].get().set_value(current_point_.positions[i]);
  }
  if (has_velocity_command_interface_)
  {
    for (size_t i=0; i<current_point_.positions.size();i++)
      (void)this->joint_command_interface_[1][i].get().set_value(current_point_.velocities[i]);
  }
  if (has_acceleration_command_interface_)
  {
    for (size_t i=0; i<current_point_.positions.size();i++)
      (void)this->joint_command_interface_[2][i].get().set_value(current_point_.accelerations[i]);
  }

  td_.scaled_time = rclcpp::Duration::from_seconds(td_.scaled_time.seconds() + period.seconds() * speed_ovr);
  td_.time        = rclcpp::Duration::from_seconds(td_.time.seconds() + period.seconds());

  // Publishers are try-locked: if one is still busy with the previous message, this sample is skipped.
  if (scaled_time_pub_ && scaled_time_pub_->trylock())
  {
    scaled_time_pub_->msg_.data = td_.scaled_time.seconds();
    scaled_time_pub_->unlockAndPublish();
  }

  if (execution_ratio_pub_ && execution_ratio_pub_->trylock())
  {
    const double trj_time = microinterpolator_->trjTime().seconds();
    execution_ratio_pub_->msg_.data = (trj_time > 0) ? std::min(1.0, td_.scaled_time.seconds() / trj_time) : 1.0;
    execution_ratio_pub_->unlockAndPublish();
  }

  // unscaled joint target (interpolation errors are already reported above)
  microinterpolator_->interpolate(td_.scaled_time,unscaled_pnt_,1);
  if (unscaled_joint_target_pub_ && unscaled_joint_target_pub_->trylock())
  {
    auto& msg = unscaled_joint_target_pub_->msg_;
    if (msg.name.size() != joint_names_.size())
      msg.name = joint_names_;
    msg.position     = unscaled_pnt_.positions;
    msg.velocity     = unscaled_pnt_.velocities;
    msg.effort       = unscaled_pnt_.effort;
    msg.header.stamp = time;
    unscaled_joint_target_pub_->unlockAndPublish();
  }

  state_desired_ = current_point_;
  state_current_ = current_point_;
#if RCLCPP_VERSION_MAJOR >= 28  // Jazzy: publish_state() also takes the time
  publish_state(time, state_desired_, state_current_, state_error_);
#else
  (void)time;
  publish_state(state_desired_, state_current_, state_error_);
#endif

  const int64_t update_ns = elapsedNs(begin, Clock::now());
  storeMax(max_update_ns_, update_ns);

  if (debug_enabled_)
  {
    async_log_->push(AsyncLogger::Level::Debug,
                     printCurrentPos() + " time: " + std::to_string(td_.time.seconds()) +
                     " speed ovr: " + std::to_string(speed_ovr) + " UPDATE time: " +
                     std::to_string(update_ns / 1000) + " [microseconds]");
  }

  return controller_interface::return_type::OK;
}

void ScaledFjtController::reportTiming()
{
  // Called with goal_mtx_ held (reads last_goal_time_).
  const Clock::time_point now = Clock::now();
  if (now - last_timing_report_ < std::chrono::seconds(1))
    return;
  last_timing_report_ = now;

  const int64_t max_update = max_update_ns_.exchange(0);
  const int64_t max_period = max_period_ns_.exchange(0);
  const uint32_t late = late_cycles_.exchange(0);

  // Warn on late cycles; report at Info for a few seconds after a goal arrives; Debug otherwise.
  AsyncLogger::Level level = AsyncLogger::Level::Debug;
  if (late > 0)
    level = AsyncLogger::Level::Warn;
  else if (last_goal_time_ != Clock::time_point{} && now - last_goal_time_ < std::chrono::seconds(3))
    level = AsyncLogger::Level::Info;
  if (level == AsyncLogger::Level::Debug && !async_log_->debugEnabled())
    return;

  std::string msg = "update() timing, last second: max duration " + toMs(max_update) +
                    ", max period " + toMs(max_period);
  if (late_threshold_ns_ > 0)
    msg += " (nominal " + toMs(static_cast<int64_t>(late_threshold_ns_ / 1.5)) + "), late cycles: " +
           std::to_string(late);
  async_log_->push(level, msg);
}

void ScaledFjtController::goalMonitorCallback()
{
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  reportTiming();
  if (!goal_handle_)
    return;

  // Cancelled, aborted or preempted elsewhere (e.g. by the base class on deactivation).
  if (!goal_handle_->is_executing())
  {
    goal_handle_.reset();
    return;
  }

  if (finished_goal_id_.load() == goal_id_)
  {
    auto result = std::make_shared<FollowJTrajAction::Result>();
    result->error_code = result->SUCCESSFUL;
    goal_handle_->succeed(result);
    goal_handle_.reset();
    async_log_->push(AsyncLogger::Level::Info, "Goal succeeded");
  }
}

rclcpp_action::GoalResponse ScaledFjtController::goal_received_callback(
    const rclcpp_action::GoalUUID & uuid
    , std::shared_ptr<const FollowJTrajAction::Goal> goal
    )
{
  const Clock::time_point t0 = Clock::now();
  auto ret = JointTrajectoryController::goal_received_callback(uuid,goal);
  validation_ns_.store(elapsedNs(t0, Clock::now()));
  return ret;
}

rclcpp_action::CancelResponse ScaledFjtController::goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle
    )
{
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  std::shared_ptr<Microinterpolator> old_interpolator;  // destroyed after mtx_ is released
  {
    std::lock_guard<std::mutex> lock(mtx_);

    // Keep the robot at the current position. Done under the lock so the hold point is exactly the
    // last commanded one; it is a single point, so this is fast.
    current_point_.time_from_start = rclcpp::Duration::from_seconds(0.0);
    trajectory_msgs::msg::JointTrajectory trj;
    trj.points.push_back(current_point_);
    auto hold = std::make_shared<Microinterpolator>();
    hold->setSplineOrder(spline_order_);
    hold->setTrajectory(std::move(trj));
    old_interpolator = std::move(microinterpolator_);
    microinterpolator_ = std::move(hold);
    active_goal_id_ = 0;
  }
  if (goal_handle_ == goal_handle)
    goal_handle_.reset();  // the base class sends the "canceled" result
  async_log_->push(AsyncLogger::Level::Info, "Goal cancelled, holding the current position");

  auto ret = JointTrajectoryController::goal_cancelled_callback(goal_handle);
  return ret;
}

void ScaledFjtController::goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  const Clock::time_point t_start = Clock::now();
  JointTrajectoryController::goal_accepted_callback(goal_handle);
  const Clock::time_point t_base = Clock::now();

  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
  last_goal_time_ = t_start;

  // Everything proportional to the trajectory length is done here, without holding mtx_, so the
  // real-time update() loop keeps running while a long trajectory is being prepared.
  trajectory_msgs::msg::JointTrajectory sorted_trj;
  if (!this->sort_trajectory(joint_names_, goal_handle->get_goal()->trajectory, sorted_trj))
  {
    async_log_->push(AsyncLogger::Level::Error, "Names are different");
    auto result = std::make_shared<FollowJTrajAction::Result>();
    result->error_code = result->INVALID_JOINTS;
    goal_handle->abort(result);

    return;
  }

  const Clock::time_point t_sort = Clock::now();

  const size_t n_points = sorted_trj.points.size();
  auto new_interpolator = std::make_shared<Microinterpolator>();
  new_interpolator->setSplineOrder(spline_order_);
  new_interpolator->setTrajectory(std::move(sorted_trj));
  const Clock::time_point t_load = Clock::now();

  if (async_log_->debugEnabled())
    async_log_->push(AsyncLogger::Level::Debug, "active goal " + new_interpolator->summary(3, 3));
  const double duration = new_interpolator->trjTime().seconds();

  const uint64_t id = ++goal_counter_;
  std::shared_ptr<Microinterpolator> old_interpolator;  // destroyed after mtx_ is released
  Clock::time_point t_lock_acquired;
  const Clock::time_point t_before_lock = Clock::now();
  {
    std::lock_guard<std::mutex> lock(mtx_);
    t_lock_acquired = Clock::now();
    old_interpolator = std::move(microinterpolator_);
    microinterpolator_ = std::move(new_interpolator);
    td_.scaled_time = rclcpp::Duration::from_seconds(0.0);
    td_.time        = rclcpp::Duration::from_seconds(0.0);
    active_goal_id_ = id;
  }
  const Clock::time_point t_swap = Clock::now();
  old_interpolator.reset();  // frees the previous trajectory, timed separately
  const Clock::time_point t_free = Clock::now();
  goal_handle_ = goal_handle;
  goal_id_ = id;

  async_log_->push(AsyncLogger::Level::Info,
                   "Trajectory set: " + std::to_string(n_points) + " points, duration " +
                   std::to_string(duration) + " s. Preparation: validation " + toMs(validation_ns_.load()) +
                   ", base class " + toMs(elapsedNs(t_start, t_base)) +
                   ", sort " + toMs(elapsedNs(t_base, t_sort)) +
                   ", load " + toMs(elapsedNs(t_sort, t_load)) +
                   ", lock wait " + toMs(elapsedNs(t_before_lock, t_lock_acquired)) +
                   ", swap " + toMs(elapsedNs(t_lock_acquired, t_swap)) +
                   ", free old " + toMs(elapsedNs(t_swap, t_free)));
}

void ScaledFjtController::SpeedOvrCb(const std_msgs::msg::Int16& msg, const std::string& topic)
{
  double ovr;
  if (msg.data>100)
    ovr=1.0;
  else if (msg.data<0)
    ovr=0.0;
  else
    ovr=msg.data*0.01;

  double global_override = 1.0;
  {
    std::lock_guard<std::mutex> lock(speed_ovr_mtx_);
    speed_ovr_map_[topic]=ovr;
    for (const auto& p: speed_ovr_map_)
      global_override*=p.second;
  }
  speed_ovr_.store(global_override);

  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"ovr = "  << msg.data<<" global ovr = "<<global_override);
}

}  // namespace scaled_fjt_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(scaled_fjt_controller::ScaledFjtController, controller_interface::ControllerInterface)
