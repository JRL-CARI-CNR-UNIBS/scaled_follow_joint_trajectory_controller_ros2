#include <scaled_fjt_controller/scaled_fjt_controller.hpp>

#include <memory>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"

namespace scaled_fjt_controller
{

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

controller_interface::CallbackReturn ScaledFjtController::on_init()
{
  return JointTrajectoryController::on_init();
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

  speed_ovr_ = 1.0;
  // TODO: from param

  std::string speed_ovr_topic ;
  if (!get_node()->has_parameter("speed_ovr_topic"))
    speed_ovr_topic = "/speed_ovr";
  else
    speed_ovr_topic = get_node()->get_parameter("speed_ovr_topic").as_string();

  speed_ovr_sub_ = get_node()->create_subscription<std_msgs::msg::Int16>(
                     speed_ovr_topic,
                     10,
                     std::bind(&ScaledFjtController::SpeedOvrCb,
                               this,
                               std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
                     get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
                     get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
                     std::string(get_node()->get_name()) + "/follow_joint_trajectory",
                     std::bind(&ScaledFjtController::goal_received_callback, this, std::placeholders::_1, std::placeholders::_2),
                     std::bind(&ScaledFjtController::goal_cancelled_callback, this, std::placeholders::_1),
                     std::bind(&ScaledFjtController::goal_accepted_callback, this, std::placeholders::_1));


  current_point_.time_from_start = rclcpp::Duration::from_seconds(0.0);
  current_point_.positions.resize(this->dof_, 0);
  current_point_.velocities.resize(this->dof_, 0);
  current_point_.accelerations.resize(this->dof_, 0);
  current_point_.effort.resize(this->dof_, 0);
  joint_names_.resize(this->dof_,"");

  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"this->joint_state_interface_[0].size = "<< this->joint_state_interface_[0].size());

  std::string delimiter = "/position";
  for (size_t i=0; i<current_point_.positions.size();i++)
  {
    double jpos = this->joint_state_interface_[0][i].get().get_value();
    current_point_.positions[i] = jpos;

    joint_names_.at(i) = this->joint_state_interface_[0][i].get().get_name();
    joint_names_.at(i) = joint_names_.at(i).substr(0, joint_names_[i].find(delimiter));
  }

  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"starting point = \n"<< trajectory_msgs::msg::to_yaml(current_point_));

  trj_.joint_names = joint_names_;
  trj_.points.clear();
  trj_.points.push_back(current_point_);

  td_.scaled_time = rclcpp::Duration::from_seconds(0.0);
  td_.time        = rclcpp::Duration::from_seconds(0.0);

  microinterpolator_.reset(new Microinterpolator());
  microinterpolator_->setTrajectory(trj_);

  int spline_order=1;   // TODO :: from params
  microinterpolator_->setSplineOrder(spline_order);

  return ret;
}

controller_interface::return_type ScaledFjtController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if( !microinterpolator_->interpolate(td_.scaled_time,current_point_,speed_ovr_) )
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),"something wrong in interpolation.");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),"scaled time     = "  << td_.scaled_time.seconds());
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),"global override = "  << speed_ovr_);
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),"current point   = "  << trajectory_msgs::msg::to_yaml(current_point_));
  }

  td_.scaled_time = rclcpp::Duration::from_seconds(td_.scaled_time.seconds() + period.seconds() * speed_ovr_);
  td_.time        = rclcpp::Duration::from_seconds(td_.time.seconds() + period.seconds());
  
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"current point   = "  << trajectory_msgs::msg::to_yaml(current_point_));
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"td_.scaled_time   = "  << td_.scaled_time.seconds());
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"td_.time   = "  << td_.time.seconds());
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"speed ovr  = "  << speed_ovr_);

  for (size_t i=0; i<current_point_.positions.size();i++)
    this->joint_command_interface_[0][i].get().set_value(current_point_.positions[i]);

  return controller_interface::return_type::OK;
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
{return JointTrajectoryController::goal_cancelled_callback(goal_handle);}

void ScaledFjtController::goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  JointTrajectoryController::goal_accepted_callback(goal_handle);
  goal_handle_ = goal_handle;

  if (!this->sort_trajectory(joint_names_, goal_handle->get_goal()->trajectory, trj_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Names are different");
    auto result = std::make_shared<FollowJTrajAction::Result>();
    result->error_code = result->INVALID_JOINTS;
    goal_handle_->abort(result);

    return;
  }

  microinterpolator_->setTrajectory(trj_);
  td_.scaled_time = rclcpp::Duration::from_seconds(0.0);
  td_.time        = rclcpp::Duration::from_seconds(0.0);
}

void ScaledFjtController::SpeedOvrCb(const std_msgs::msg::Int16 ovr)
{
  speed_ovr_=ovr.data*0.01;
  RCLCPP_DEBUG_STREAM(get_node()->get_logger(),"ovr = "  << ovr.data);
}

} 

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(scaled_fjt_controller::ScaledFjtController, controller_interface::ControllerInterface)
