// Copyright 2024-2026 CARI-JRL (CNR-STIIMA / University of Brescia), SUPSI, Politecnico di Milano and contributors
// SPDX-License-Identifier: Apache-2.0

#include <scaled_fjt_controller/microinterpolator.h>

#include <algorithm>
#include <sstream>

Microinterpolator::Microinterpolator()
{
  order_=1;
  trj_set_=false;
}

void Microinterpolator::setSplineOrder(const unsigned int& order)
{
  if (order<5)
    order_=order;
  else
    RCLCPP_WARN_STREAM(rclcpp::get_logger("microinterpolator"), "Interpolation order should be less or equal to 4");
}

bool Microinterpolator::setTrajectory(const trajectory_msgs::msg::JointTrajectory& trj)
{
  trj_=trj;
  return init();
}

bool Microinterpolator::setTrajectory(trajectory_msgs::msg::JointTrajectory&& trj)
{
  trj_=std::move(trj);
  return init();
}

bool Microinterpolator::init()
{
  // Cache the waypoint times as seconds, so interpolate() does not convert messages at every cycle.
  times_.resize(trj_.points.size());
  for (size_t i=0;i<trj_.points.size();i++)
    times_[i]=rclcpp::Duration(trj_.points[i].time_from_start).seconds();
  segment_=1;
  trj_set_ = true;
  last_error_=nullptr;
  return trj_.points.size()>0;
}

namespace
{
void printPoint(std::ostringstream& ss, size_t idx, const trajectory_msgs::msg::JointTrajectoryPoint& p)
{
  auto vec=[&ss](const char* name, const std::vector<double>& v)
  {
    ss << " " << name << "=[";
    for (size_t i=0;i<v.size();i++)
      ss << (i ? ", " : "") << v[i];
    ss << "]";
  };
  ss << "\n  #" << idx << " t=" << rclcpp::Duration(p.time_from_start).seconds();
  vec("pos", p.positions);
  vec("vel", p.velocities);
  vec("acc", p.accelerations);
}
}  // namespace

std::string Microinterpolator::summary(size_t n_head, size_t n_tail) const
{
  std::ostringstream ss;
  const size_t n=trj_.points.size();
  ss << "trajectory: " << n << " points, duration " << (n ? times_.back() : 0.0) << " s";
  for (size_t i=0;i<n;i++)
  {
    if (i==n_head && n>n_head+n_tail)
    {
      ss << "\n  ... (" << n-n_head-n_tail << " points omitted)";
      if (n_tail == 0)
        break;
      i=n-n_tail;
    }
    printPoint(ss, i, trj_.points[i]);
  }
  return ss.str();
}

bool Microinterpolator::interpolate(const rclcpp::Duration& time, trajectory_msgs::msg::JointTrajectoryPoint& pnt, const double& scaling)
{
  // Called from the real-time loop: no logging and no searches over the whole trajectory here.
  if (!trj_set_)
  {
    last_error_="trajectory is not set";
    return false;
  }

  if (trj_.points.size()==0)
  {
    last_error_="trajectory is empty";
    return false;
  }

  const double time_s=time.seconds();

  if (time_s<times_.front())
  {
    pnt=trj_.points.at(0);
    pnt.effort.resize(trj_.points.at(0).positions.size(),0);
    last_error_="negative time (before the first trajectory point)";
    return false;
  }

  if (time_s>=times_.back())
  {
    unsigned int nAx=trj_.points.back().positions.size();
    pnt=trj_.points.back();
    pnt.velocities.assign(nAx,0);
    pnt.accelerations.assign(nAx,0);
    pnt.effort.resize(nAx,0);
    return true;
  }

  // Find the segment [iPnt-1, iPnt) containing time. Time normally moves forward, so start from the
  // segment used in the previous call instead of scanning from the first point.
  if (segment_<1 || segment_>=times_.size() || time_s<times_[segment_-1])
    segment_=1;
  while (time_s>=times_[segment_])  // terminates: time_s < times_.back()
    segment_++;
  const size_t iPnt=segment_;

  unsigned int nAx=trj_.points.at(iPnt).positions.size();
  pnt.positions.resize(nAx,0);
  pnt.velocities.resize(nAx,0);
  pnt.accelerations.resize(nAx,0);
  pnt.effort.resize(nAx,0);
  pnt.time_from_start=time;
  double delta_time=std::max(1.0e-6, times_[iPnt]-times_[iPnt-1]);
  double t=time_s-times_[iPnt-1];
  double ratio=t/delta_time;
  for (unsigned int iAx=0;iAx<nAx;iAx++)
  {
    //spline order
    if (order_==0)
    {
      pnt.positions.at(iAx)=trj_.points.at(iPnt-1).positions.at(iAx)+ratio*(trj_.points.at(iPnt).positions.at(iAx)-trj_.points.at(iPnt-1).positions.at(iAx));
      pnt.velocities.at(iAx)=(trj_.points.at(iPnt).positions.at(iAx)-trj_.points.at(iPnt-1).positions.at(iAx))/delta_time;
    }
    else if (order_==1)
    {
      double& p0_1=trj_.points.at(iPnt-1).positions.at(iAx);
      double& p0_2=trj_.points.at(iPnt-1).velocities.at(iAx);
      double& pf_1=trj_.points.at(iPnt).positions.at(iAx);
      double& pf_2=trj_.points.at(iPnt).velocities.at(iAx);
      
      double c1 = p0_1;
      double c2 = p0_2;
      double c3 = -1.0/(delta_time*delta_time)*(p0_1*3.0-pf_1*3.0+delta_time*p0_2*2.0+delta_time*pf_2);
      double c4 = 1.0/(delta_time*delta_time*delta_time)*(p0_1*2.0-pf_1*2.0+delta_time*p0_2+delta_time*pf_2);
      
      pnt.positions.at(iAx)     = c1+c2*t+c3*(t*t)+c4*(t*t*t);
      pnt.velocities.at(iAx)    = c2+c3*t*2.0+c4*(t*t)*3.0;
      pnt.accelerations.at(iAx) = c3*2.0+c4*t*6.0;
    }
    else if (order_==2)
    {
      double& p0_1=trj_.points.at(iPnt-1).positions.at(iAx);
      double& p0_2=trj_.points.at(iPnt-1).velocities.at(iAx);
      double& p0_3=trj_.points.at(iPnt-1).accelerations.at(iAx);
      double& pf_1=trj_.points.at(iPnt).positions.at(iAx);
      double& pf_2=trj_.points.at(iPnt).velocities.at(iAx);
      double& pf_3=trj_.points.at(iPnt).accelerations.at(iAx);
      
      double c1 = p0_1;
      double c2 = p0_2;
      double c3 = p0_3*(1.0/2.0);
      double c4 = 1.0/(delta_time*delta_time*delta_time)*(p0_1*2.0E1-pf_1*2.0E1+delta_time*p0_2*1.2E1+delta_time*pf_2*8.0+(delta_time*delta_time)*p0_3*3.0-(delta_time*delta_time)*pf_3)*(-1.0/2.0);
      double c5 = 1.0/(delta_time*delta_time*delta_time*delta_time)*(p0_1*3.0E1-pf_1*3.0E1+delta_time*p0_2*1.6E1+delta_time*pf_2*1.4E1+(delta_time*delta_time)*p0_3*3.0-(delta_time*delta_time)*pf_3*2.0)*(1.0/2.0);
      double c6 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.2E1-pf_1*1.2E1+delta_time*p0_2*6.0+delta_time*pf_2*6.0+(delta_time*delta_time)*p0_3-(delta_time*delta_time)*pf_3)*(-1.0/2.0);
      
      
      pnt.positions.at(iAx)     = c1+c2*t+c3*(t*t)+c4*(t*t*t)+c5*(t*t*t*t)+c6*(t*t*t*t*t);
      pnt.velocities.at(iAx)    = c2+c3*t*2.0+c4*(t*t)*3.0+c5*(t*t*t)*4.0+c6*(t*t*t*t)*5.0;
      pnt.accelerations.at(iAx) = c3*2.0+c4*t*6.0+c5*(t*t)*1.2E1+c6*(t*t*t)*2.0E1;
    }
    else if (order_==3)
    {
      double& p0_1=trj_.points.at(iPnt-1).positions.at(iAx);
      double& p0_2=trj_.points.at(iPnt-1).velocities.at(iAx);
      double& p0_3=trj_.points.at(iPnt-1).accelerations.at(iAx);
      double& pf_1=trj_.points.at(iPnt).positions.at(iAx);
      double& pf_2=trj_.points.at(iPnt).velocities.at(iAx);
      double& pf_3=trj_.points.at(iPnt).accelerations.at(iAx);
      // initial and final jerks set equal to zero

      double c1 = p0_1;
      double c2 = p0_2;
      double c3 = p0_3*(1.0/2.0);
      double c4 = 0.0;
      double c5 = 1.0/(delta_time*delta_time*delta_time*delta_time)*(p0_1*1.4E1-pf_1*1.4E1+delta_time*p0_2*8.0+delta_time*pf_2*6.0+(delta_time*delta_time)*p0_3*2.0-(delta_time*delta_time)*pf_3)*(-5.0/2.0);
      double c6 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*8.4E1-pf_1*8.4E1+delta_time*p0_2*4.5E1+delta_time*pf_2*3.9E1+(delta_time*delta_time)*p0_3*1.0E1-(delta_time*delta_time)*pf_3*7.0);
      double c7 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.4E2-pf_1*1.4E2+delta_time*p0_2*7.2E1+delta_time*pf_2*6.8E1+(delta_time*delta_time)*p0_3*1.5E1-(delta_time*delta_time)*pf_3*1.3E1)*(-1.0/2.0);
      double c8 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.0E1-pf_1*1.0E1+delta_time*p0_2*5.0+delta_time*pf_2*5.0+(delta_time*delta_time)*p0_3-(delta_time*delta_time)*pf_3)*2.0;
      
      
      pnt.positions.at(iAx)     = c1+c2*t+c3*(t*t)+c4*(t*t*t)+c5*(t*t*t*t)+c6*(t*t*t*t*t)+c7*(t*t*t*t*t*t)+c8*(t*t*t*t*t*t*t);
      pnt.velocities.at(iAx)    = c2+c3*t*2.0+c4*(t*t)*3.0+c5*(t*t*t)*4.0+c6*(t*t*t*t)*5.0+c7*(t*t*t*t*t)*6.0+c8*(t*t*t*t*t*t)*7.0;
      pnt.accelerations.at(iAx) = c3*2.0+c4*t*6.0+c5*(t*t)*1.2E1+c6*(t*t*t)*2.0E1+c7*(t*t*t*t)*3.0E1+c8*(t*t*t*t*t)*4.2E1;
    }
    else if (order_==4)
    {
      double& p0_1=trj_.points.at(iPnt-1).positions.at(iAx);
      double& p0_2=trj_.points.at(iPnt-1).velocities.at(iAx);
      double& p0_3=trj_.points.at(iPnt-1).accelerations.at(iAx);
      double& pf_1=trj_.points.at(iPnt).positions.at(iAx);
      double& pf_2=trj_.points.at(iPnt).velocities.at(iAx);
      double& pf_3=trj_.points.at(iPnt).accelerations.at(iAx);
      // initial and final jerks and snaps set equal to zero

      double c1 = p0_1;
      double c2 = p0_2;
      double c3 = p0_3*(1.0/2.0);
      double c4 = 0.0;
      double c5 = 0.0;
      double c6 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*3.6E1-pf_1*3.6E1+delta_time*p0_2*2.0E1+delta_time*pf_2*1.6E1+(delta_time*delta_time)*p0_3*5.0-(delta_time*delta_time)*pf_3*3.0)*(-7.0/2.0);
      double c7 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.2E2-pf_1*1.2E2+delta_time*p0_2*6.4E1+delta_time*pf_2*5.6E1+(delta_time*delta_time)*p0_3*1.5E1-(delta_time*delta_time)*pf_3*1.1E1)*(7.0/2.0);
      double c8 = -1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*5.4E2-pf_1*5.4E2+delta_time*p0_2*2.8E2+delta_time*pf_2*2.6E2+(delta_time*delta_time)*p0_3*6.3E1-(delta_time*delta_time)*pf_3*5.3E1);
      double c9 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.26E2-pf_1*1.26E2+delta_time*p0_2*6.4E1+delta_time*pf_2*6.2E1+(delta_time*delta_time)*p0_3*1.4E1-(delta_time*delta_time)*pf_3*1.3E1)*(5.0/2.0);
      double c10 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*2.8E1-pf_1*2.8E1+delta_time*p0_2*1.4E1+delta_time*pf_2*1.4E1+(delta_time*delta_time)*p0_3*3.0-(delta_time*delta_time)*pf_3*3.0)*(-5.0/2.0);
      
      
      pnt.positions.at(iAx)     = c1+c2*t+c3*(t*t)+c4*(t*t*t)+c5*(t*t*t*t)+c6*(t*t*t*t*t)+c7*(t*t*t*t*t*t)+c8*(t*t*t*t*t*t*t)+c9*(t*t*t*t*t*t*t*t)+c10*(t*t*t*t*t*t*t*t*t);
      pnt.velocities.at(iAx)    = c2+c3*t*2.0+c4*(t*t)*3.0+c5*(t*t*t)*4.0+c6*(t*t*t*t)*5.0+c7*(t*t*t*t*t)*6.0+c8*(t*t*t*t*t*t)*7.0+c9*(t*t*t*t*t*t*t)*8.0+c10*(t*t*t*t*t*t*t*t)*9.0;
      pnt.accelerations.at(iAx) = c3*2.0+c4*t*6.0+c5*(t*t)*1.2E1+c6*(t*t*t)*2.0E1+c7*(t*t*t*t)*3.0E1+c8*(t*t*t*t*t)*4.2E1+c9*(t*t*t*t*t*t)*5.6E1+c10*(t*t*t*t*t*t*t)*7.2E1;
    }
    
    pnt.velocities.at(iAx)    *= scaling ;
    pnt.accelerations.at(iAx) *= scaling*scaling;
  }

  return true;
}

rclcpp::Duration Microinterpolator::trjTime()
{
  if (trj_set_ && !times_.empty())
    return rclcpp::Duration::from_seconds(times_.back());
  else
    return rclcpp::Duration::from_seconds(0.0);
}
