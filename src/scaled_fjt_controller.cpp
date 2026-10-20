// Copyright 2024-2026 CARI-JRL (CNR-STIIMA / University of Brescia), SUPSI, Politecnico di Milano and contributors
// SPDX-License-Identifier: Apache-2.0

#include <scaled_fjt_controller/scaled_fjt_controller.hpp>

#include <algorithm>
#include <memory>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include <rclcpp/version.h>

namespace scaled_fjt_controller
{

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

  trj_.joint_names = joint_names_;
  trj_.points.clear();
  trj_.points.push_back(current_point_);

  auto hold = std::make_shared<Microinterpolator>();
  hold->setSplineOrder(spline_order_);
  hold->setTrajectory(trj_);
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
  std::lock_guard<std::mutex> lock(mtx_); // goal callbacks hold it only to swap the interpolator

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
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

  if (debug_enabled_)
  {
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    async_log_->push(AsyncLogger::Level::Debug,
                     printCurrentPos() + " time: " + std::to_string(td_.time.seconds()) +
                     " speed ovr: " + std::to_string(speed_ovr) + " UPDATE time: " +
                     std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) +
                     " [microseconds]");
  }

  state_desired_ = current_point_;
  state_current_ = current_point_;
#if RCLCPP_VERSION_MAJOR >= 28  // Jazzy: publish_state() also takes the time
  publish_state(time, state_desired_, state_current_, state_error_);
#else
  (void)time;
  publish_state(state_desired_, state_current_, state_error_);
#endif

  return controller_interface::return_type::OK;
}

void ScaledFjtController::goalMonitorCallback()
{
  std::lock_guard<std::mutex> goal_lock(goal_mtx_);
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
  auto ret = JointTrajectoryController::goal_received_callback(uuid,goal);
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
  JointTrajectoryController::goal_accepted_callback(goal_handle);

  std::lock_guard<std::mutex> goal_lock(goal_mtx_);

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

  auto new_interpolator = std::make_shared<Microinterpolator>();
  new_interpolator->setSplineOrder(spline_order_);
  new_interpolator->setTrajectory(sorted_trj);

  async_log_->push(AsyncLogger::Level::Info,
                   "Trajectory set: " + std::to_string(sorted_trj.points.size()) + " points, duration " +
                   std::to_string(new_interpolator->trjTime().seconds()) + " s");
  if (async_log_->debugEnabled())
    async_log_->push(AsyncLogger::Level::Debug, "active goal " + new_interpolator->summary(3, 3));

  const uint64_t id = ++goal_counter_;
  std::shared_ptr<Microinterpolator> old_interpolator;  // destroyed after mtx_ is released
  {
    std::lock_guard<std::mutex> lock(mtx_);
    old_interpolator = std::move(microinterpolator_);
    microinterpolator_ = std::move(new_interpolator);
    td_.scaled_time = rclcpp::Duration::from_seconds(0.0);
    td_.time        = rclcpp::Duration::from_seconds(0.0);
    active_goal_id_ = id;
  }
  goal_handle_ = goal_handle;
  goal_id_ = id;
  trj_ = std::move(sorted_trj);
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
