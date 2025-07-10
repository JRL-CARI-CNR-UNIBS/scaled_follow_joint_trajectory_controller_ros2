// Copyright (c) 2021 ros2_control Development Team
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

#include "scaled_fjt_controller/scaled_fjt_controller.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "angles/angles.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"


namespace scaled_fjt_controller
{





controller_interface::CallbackReturn ScaledFjtController::on_init()
{
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Declare the custom parameter
  get_node()->declare_parameter<int8_t>("spline_order", 1);
  get_node()->declare_parameter< std::vector<std::string> >("speed_ovr_topics", {
    "/speed_ovr",
    "/safe_ovr"
  });
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ScaledFjtController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf = JointTrajectoryController::command_interface_configuration();

  return conf;
}

controller_interface::InterfaceConfiguration ScaledFjtController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf = JointTrajectoryController::state_interface_configuration();

  return conf;
}

controller_interface::return_type ScaledFjtController::update(
  const rclcpp::Time&, const rclcpp::Duration& period)
{
  rclcpp::Time begin = get_node()->now();
  
  double _updated_speed_over = interpolate(period);

  if(goal_handle_ && goal_handle_->is_executing() && (td_.scaled_time-trj_.points.back().time_from_start).seconds()>=0)
  {
    bool tolerance_violated_while_moving;
    bool goal_tolerance_violated;
    bool within_goal_time;
    this->check_tolerances(tolerance_violated_while_moving, goal_tolerance_violated, within_goal_time);

    if (!goal_tolerance_violated)
    {
      auto result = std::make_shared<FollowJTrajAction::Result>();
      result->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
      result->set__error_string("Goal successfully reached!");
      goal_handle_->succeed(result);
      goal_handle_ = nullptr;
    }
    else if (!within_goal_time)
    {
      const std::string error_string = "Aborted due to goal_time_tolerance exceeding";
      auto result = std::make_shared<FollowJTrajAction::Result>();
      result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
      result->set__error_string(error_string);
      goal_handle_->abort(result);
      goal_handle_ = nullptr;
    }
  }

  if(!update_commands())
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),"Failed to update commands.");
    return controller_interface::return_type::ERROR;
  }

  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),printCurrentPos());

  publish_unscaled_js_target();

  publish_state(begin,state_desired_, state_current_, state_error_);

  rclcpp::Time end = get_node()->now();
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"UPDATE time:  = " << (end - begin).seconds() << "[seconds]" );

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ScaledFjtController::on_configure(const rclcpp_lifecycle::State & state)
{
  // workaround to superimpose the  HW_IF_VELOCITY, HW_IF_ACCELERATION and HW_IF_EFFORT
  // interfaces on the HW_IF_OFFSET_VELOCITY, HW_IF_OFFSET_ACCELERATION and
  // HW_IF_OFFSET_EFFORT interfaces, if they are present in the command_interfaces parameter
  // of the controller. This is needed to allow the controller to work with the
  // hardware_interface::ScaledJointVelocityController, which uses the offset interfaces
  // to scale the velocity, acceleration and effort commands.
  // This is a workaround for the fact that the hardware_interface::ScaledJointVelocityController
  // does not support the offset interfaces, but only the standard interfaces.
  
  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  auto _contains_interface_type = [&](
  const std::vector<std::string> & interface_type_list, const std::string & interface_type)
  {
    return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
          interface_type_list.end();
  };

  std::vector<std::string> _command_interfaces = params_.command_interfaces;
  if( _contains_interface_type(params_.command_interfaces, scaled_fjt_controller::HW_IF_OFFSET_VELOCITY) 
  && !_contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_VELOCITY) )
  {
    _command_interfaces.push_back(hardware_interface::HW_IF_VELOCITY);
  }

  if( _contains_interface_type(params_.command_interfaces, scaled_fjt_controller::HW_IF_OFFSET_ACCELERATION) 
  && !_contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_ACCELERATION) )
  {
    _command_interfaces.push_back(hardware_interface::HW_IF_ACCELERATION);
  }

  if( _contains_interface_type(params_.command_interfaces, scaled_fjt_controller::HW_IF_OFFSET_EFFORT) 
  && !_contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_EFFORT) )
  {
    _command_interfaces.push_back(hardware_interface::HW_IF_EFFORT);
  }

  if( _command_interfaces.size() != params_.command_interfaces.size() )
  {
    get_node()->set_parameter(rclcpp::Parameter("command_interfaces", _command_interfaces)); 
  }

  
  auto ret = JointTrajectoryController::on_configure(state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Workaround. 
  // The action_server is created in the JointTrajectoryController::on_configure method,
  // but we need to recreate it here to ensure that the action server is created with the
  // correct node interfaces. 
  // The issue is that the 'JointTrajectoryController::goal_accepted_callback' is not virtual, and
  // it cannot be overridden by ths ScaledFjtController::goal_accepted_callabck.
  action_server_.reset();
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
        get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
        get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
        std::string(get_node()->get_name()) + "/follow_joint_trajectory",
        std::bind(&ScaledFjtController::goal_received_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ScaledFjtController::goal_cancelled_callback, this, std::placeholders::_1),
        std::bind(&ScaledFjtController::goal_accepted_callback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ScaledFjtController::on_activate(const rclcpp_lifecycle::State& state)
{
  auto ret = JointTrajectoryController::on_activate(state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Management of the further parameters
  std::vector<std::string> speed_ovr_topics;
  if (!get_node()->has_parameter("speed_ovr_topics.topics"))
  {
    speed_ovr_topics.push_back("/speed_ovr");
    speed_ovr_topics.push_back("/safe_ovr");
  }
  else
  {
    speed_ovr_topics = get_node()->get_parameter("speed_ovr_topics.topics").as_string_array();
  }

  speed_ovr_topics_policy_ = SpeedOvrTopicPolicy::MULTIPLY;
  if (get_node()->has_parameter("speed_ovr_topics.policy"))
  {
    std::string policy = get_node()->get_parameter("speed_ovr_topics.policy").as_string();
    if (policy.compare("MULTIPLY")==0)
    {
      speed_ovr_topics_policy_ = SpeedOvrTopicPolicy::MULTIPLY;
    }
    else if (policy.compare("MINIMUM")==0)
    {
      speed_ovr_topics_policy_ = SpeedOvrTopicPolicy::MINIMUM;
    }
    else if (policy.compare("MAXIMUM")==0)
    {
      speed_ovr_topics_policy_ = SpeedOvrTopicPolicy::MAXIMUM;
    }
    else if (policy.compare("AVERAGE")==0)
    {
      speed_ovr_topics_policy_ = SpeedOvrTopicPolicy::AVERAGE;
    }
    else
    {
      RCLCPP_ERROR(get_node()->get_logger(),"Topic policy %s is unknown. Using MULTIPLY as default.", policy.c_str());
    }
  }

  int spline_order ;
  if (!get_node()->has_parameter("spline_order"))
  {
    RCLCPP_ERROR(this->get_node()->get_logger(),"!!! Spline has not been set, setting spline order to 1");
    spline_order = 1;
  }
  else
  {
    spline_order = get_node()->get_parameter("spline_order").as_int();
  }

  if(spline_order<1)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(),"Spline order cannot be less than 1, set equal to 1");
    spline_order = 1;
  }
  else
  {
    RCLCPP_WARN(this->get_node()->get_logger(),"Spline order is equal to %d", spline_order);
  }

  for(const std::string& topic: speed_ovr_topics)
  {
    auto cb=[this,topic](const std_msgs::msg::Int16 msg){return this->SpeedOvrCb(msg,topic);};

    speed_ovr_sub_.push_back(get_node()->create_subscription<std_msgs::msg::Int16>(topic,10,cb));
    speed_ovr_map_.insert(std::pair<std::string,double>(topic,1.0));
    RCLCPP_INFO_STREAM(this->get_node()->get_logger(),"Subscribing speed override topic: "<<topic);
  }
  speed_ovr_ = 1.0;
  // Management of the further parameters - END

  // microinterpolator cofiguration 
  // Initialize the joint state and command interfaces - the actual point is added to the trajectory (microintepolate)
  state_desired_.time_from_start = rclcpp::Duration::from_seconds(0.0);
  state_desired_.positions.resize(this->dof_, 0);
  state_desired_.velocities.resize(this->dof_, 0);
  state_desired_.accelerations.resize(this->dof_, 0);
  state_desired_.effort.resize(this->dof_, 0);
  joint_names_.resize(this->dof_,"");

  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"this->joint_state_interface_[0].size = "<< this->joint_state_interface_[0].size());

  std::string delimiter = "/position";
  for (size_t i=0; i<state_desired_.positions.size();i++)
  {
    auto  _jpos = this->joint_state_interface_[0][i].get().get_optional();
    if (_jpos)
    { 
      double jpos = _jpos.value();
      state_desired_.positions[i] = jpos;

      joint_names_.at(i) = this->joint_state_interface_[0][i].get().get_name();
      joint_names_.at(i) = joint_names_.at(i).substr(0, joint_names_[i].find(delimiter));
    }
    else
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),"Joint state interface for joint "<<i<<" is not available.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"starting point = \n"<< trajectory_msgs::msg::to_yaml(state_desired_));

  trj_.joint_names = joint_names_;
  trj_.points.clear();
  trj_.points.push_back(state_desired_);

  td_.scaled_time = rclcpp::Duration::from_seconds(0.0);
  td_.time        = rclcpp::Duration::from_seconds(0.0);

  microinterpolator_.reset(new Microinterpolator());
  microinterpolator_->setTrajectory(trj_);

  microinterpolator_->setSplineOrder(spline_order);
  // microinterpolator cofiguration - END

  // add publishers unscaled, scaled time, execution ratio
  unscaled_js_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
  unscaled_js_msg_->name = joint_names_;
  unscaled_js_msg_->effort = state_desired_.effort;
  unscaled_js_msg_->position = state_desired_.positions;
  unscaled_js_msg_->velocity = state_desired_.velocities;
  unscaled_js_msg_->header.stamp  = get_node()->get_clock()->now();

  scaled_time_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>("scaled_time", 10);
  execution_ratio_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>("execution_ratio", 10);
  unscaled_joint_target_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("unscaled_joint_target", 10);

  return controller_interface::CallbackReturn::SUCCESS;
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
  std::lock_guard<std::mutex> lock(mtx_);

  // Keep the robot at the current position
  state_desired_.time_from_start = rclcpp::Duration::from_seconds(0.0);
  trajectory_msgs::msg::JointTrajectory trj;
  trj.points.push_back(state_desired_);
  microinterpolator_->setTrajectory(trj);

  auto ret = JointTrajectoryController::goal_cancelled_callback(goal_handle);
  return ret;
}

void ScaledFjtController::goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  JointTrajectoryController::goal_accepted_callback(goal_handle);

  std::lock_guard<std::mutex> lock(mtx_);

  if (!this->sort_trajectory(joint_names_, goal_handle->get_goal()->trajectory, trj_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Names are different");
    auto result = std::make_shared<FollowJTrajAction::Result>();
    result->error_code = result->INVALID_JOINTS;
    goal_handle->abort(result);

    return;
  }

  td_.scaled_time = rclcpp::Duration::from_seconds(0.0);
  td_.time        = rclcpp::Duration::from_seconds(0.0);
  microinterpolator_->setTrajectory(trj_);

  goal_handle_ = goal_handle; //in the last line, otherwise goal_handle_->succeed() may happen in update()
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

void ScaledFjtController::SpeedOvrCb(const std_msgs::msg::Int16& msg, const std::string& topic)
{
  double ovr;
  if (msg.data>100)
    ovr=1.0;
  else if (msg.data<0)
    ovr=0.0;
  else
    ovr=msg.data*0.01;

  speed_ovr_map_.at(topic)=ovr;

  double global_override = 1.0;

  switch (speed_ovr_topics_policy_)
  {
  case MULTIPLY:
    for (const std::pair<std::string,double> p: speed_ovr_map_)
      global_override*=p.second;
    break;
  case MINIMUM:
    for (const std::pair<std::string,double> p: speed_ovr_map_)
      global_override=std::min(global_override, p.second);
    break;
  case MAXIMUM:
    for (const std::pair<std::string,double> p: speed_ovr_map_)
      global_override=std::max(global_override, p.second);
    break;
  case AVERAGE:
    for (const std::pair<std::string,double> p: speed_ovr_map_)
      global_override+=p.second;
    global_override/=speed_ovr_map_.size();
    break;
  }

  speed_ovr_mtx_.lock();
  speed_ovr_ = global_override;
  speed_ovr_mtx_.unlock();

  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"ovr = "  << msg.data<<" global ovr = "<<global_override);
}


std::string ScaledFjtController::printCurrentPos()
{
  std::string p = "time: "+std::to_string(td_.scaled_time.seconds())+ " pos: ";
  for (size_t i=0; i<state_desired_.positions.size();i++)
  {
    p = p+std::to_string(state_desired_.positions[i])+" ";
  }
  return p;
}

bool ScaledFjtController::check_tolerances(bool& tolerance_violated_while_moving,
                                           bool& outside_goal_tolerance,
                                           bool& within_goal_time)
{
  auto active_tol = active_tolerances_.readFromRT();
  tolerance_violated_while_moving = false;
  outside_goal_tolerance = false;
  within_goal_time = true;

  // Check state/goal tolerance
  for (size_t index = 0; index < dof_; ++index)
  {
    compute_error_for_joint(state_error_, index, state_current_, state_desired_);

    // Check points are within path tolerance
    if (td_.scaled_time < microinterpolator_->trjTime() &&
        !check_state_tolerance_per_joint(
        state_error_, index, active_tol->state_tolerance[index], true /* show_errors */))
    {
      tolerance_violated_while_moving = true;
    }
    // past the final point, check that we end up inside goal tolerance
    if (td_.scaled_time >= microinterpolator_->trjTime() &&
        !check_state_tolerance_per_joint(
          state_error_, index, active_tol->goal_state_tolerance[index], false /* show_errors */))
    {
      outside_goal_tolerance = true;

      if (active_tol->goal_time_tolerance != 0.0)
      {
        if ( (td_.scaled_time - microinterpolator_->trjTime()).seconds() > active_tol->goal_time_tolerance)
        {
          within_goal_time = false;
        }
      }
    }
  }

  return outside_goal_tolerance || tolerance_violated_while_moving;
}

bool ScaledFjtController::update_commands()
{
  bool ret = true;
  // send command to robot
  if (has_position_command_interface_)
  {
    for (size_t i=0; i<state_desired_.positions.size();i++)
      ret &= this->joint_command_interface_[0][i].get().set_value(state_desired_.positions[i]);
  }
  if (has_velocity_command_interface_)
  {
    for (size_t i=0; i<state_desired_.positions.size();i++)
      ret &= this->joint_command_interface_[1][i].get().set_value(state_desired_.velocities[i]);
  }
  if (has_acceleration_command_interface_)
  {
    for (size_t i=0; i<state_desired_.positions.size();i++)
      ret &= this->joint_command_interface_[2][i].get().set_value(state_desired_.accelerations[i]);
  }
  return ret;
}

double ScaledFjtController::interpolate(const rclcpp::Duration& period)
{
  std::lock_guard<std::mutex> lock(mtx_); //protect when new goal arrives. Finish the loop and eventually update the goal

  speed_ovr_mtx_.lock();
  double speed_ovr = speed_ovr_;
  speed_ovr_mtx_.unlock();

  if( !microinterpolator_->interpolate(td_.scaled_time,state_desired_,speed_ovr) )
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),"something wrong in interpolation.");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),"scaled time     = "  << td_.scaled_time.seconds());
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),"global override = "  << speed_ovr);
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),"current point   = "  << trajectory_msgs::msg::to_yaml(state_desired_));
  }

  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"current point   = "  << trajectory_msgs::msg::to_yaml(state_desired_));
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"td_.scaled_time   = "  << td_.scaled_time.seconds());
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"td_.time   = "  << td_.time.seconds());
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"speed ovr  = "  << speed_ovr);


  td_.scaled_time = rclcpp::Duration::from_seconds(td_.scaled_time.seconds() + period.seconds() * speed_ovr);
  td_.time        = rclcpp::Duration::from_seconds(td_.time.seconds() + period.seconds());

  return speed_ovr;
}

void ScaledFjtController::publish_unscaled_js_target()
{
  // publish scaled time
  std::shared_ptr<std_msgs::msg::Float64> scaled_msg(new std_msgs::msg::Float64());
  scaled_msg->data=td_.scaled_time.seconds();
  scaled_time_pub_->publish(*scaled_msg);

  // publish execution ratio
  std::shared_ptr<std_msgs::msg::Float64> ratio_msg(new std_msgs::msg::Float64());
  if (microinterpolator_->trjTime().seconds()>0)
  {
    ratio_msg->data=std::min(1.0,td_.scaled_time.seconds()/microinterpolator_->trjTime().seconds());
  }
  else
  {
    ratio_msg->data=1;
  }
  execution_ratio_pub_->publish(*ratio_msg);

  // compute unscaled joint target
  trajectory_msgs::msg::JointTrajectoryPoint unscaled_pnt;
  if( !microinterpolator_->interpolate(td_.scaled_time,unscaled_pnt,1) )
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),"something wrong in interpolation.");
  }

  // publish unscaled joint target
  unscaled_js_msg_->position      = unscaled_pnt.positions;
  unscaled_js_msg_->velocity      = unscaled_pnt.velocities;
  unscaled_js_msg_->effort        = unscaled_pnt.effort;
  unscaled_js_msg_->header.stamp  = get_node()->get_clock()->now();
  unscaled_joint_target_pub_->publish(*unscaled_js_msg_);
}

}  // namespace scaled_fjt_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(scaled_fjt_controller::ScaledFjtController, controller_interface::ControllerInterface)
