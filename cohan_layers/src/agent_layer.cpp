/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 *********************************************************************/

#include <cohan_layers/agent_layer.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#define DEFAULT_AGENT_PART cohan_msgs::TrackedSegmentType::TORSO
#define TRACKED_AGENT_SUB "/tracked_agents"
#define AGENTS_STATES_SUB "/move_base/HATebLocalPlannerROS/agents_states"

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace cohan_layers
{
void AgentLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  agents_sub_ = nh.subscribe(TRACKED_AGENT_SUB, 1, &AgentLayer::agentsCB, this);
  agents_states_sub_ = nh.subscribe(AGENTS_STATES_SUB, 1, &AgentLayer::statesCB, this);
  stopmap_srv = nh.advertiseService("shutdown_layer", &AgentLayer::shutdownCB, this);

  current_ = true;
  first_time_ = true;
  shutdown_ = false;
}

void AgentLayer::agentsCB(const cohan_msgs::TrackedAgents& agents)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  agents_ = agents;
}

void AgentLayer::statesCB(const cohan_msgs::StateArray& states){
  boost::recursive_mutex::scoped_lock lock(lock_);
  states_ = states;
  reset = false;
  last_time = ros::Time::now();
}

bool AgentLayer::shutdownCB(std_srvs::SetBoolRequest&req, std_srvs::SetBoolResponse &res){
  shutdown_ = req.data;
  if(shutdown_ == true){
    res.success = true;
    res.message = "Shutting down the agent layer costmaps..";
  }
  else{
    res.success = true;
    res.message = "Agent layer is switched on !";
  }
  return true;
}


void AgentLayer::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  boost::recursive_mutex::scoped_lock lock(lock_);

  std::string global_frame = layered_costmap_->getGlobalFrameID();
  transformed_agents_.clear();

  if((ros::Time::now()-last_time).toSec() > 1.0){
    reset = true;
    states_.agents_states.clear();
  }

  for(auto &agent : agents_.agents){
    for(auto &segment : agent.segments){
      if((segment.type == DEFAULT_AGENT_PART) && !reset){
        if(!states_.agents_states.empty() && !shutdown_){
          for(auto &a_state: states_.agents_states){
            if(a_state.track_id == agent.track_id){
              if((int)a_state.state != 0){
                AgentPoseVel agent_pose_vel;
                agent_pose_vel.header.frame_id = agents_.header.frame_id;
                agent_pose_vel.header.stamp = agents_.header.stamp;
                geometry_msgs::PoseStamped before_pose, after_pose;

                try
                {
                  before_pose.pose = segment.pose.pose;
                  before_pose.header.frame_id = agents_.header.frame_id;
                  before_pose.header.stamp = agents_.header.stamp;
                  tf_->transform(before_pose,after_pose,global_frame,ros::Duration(0.5));
                  agent_pose_vel.pose = after_pose.pose;

                  before_pose.pose.position.x += segment.twist.twist.linear.x;
                  before_pose.pose.position.y += segment.twist.twist.linear.y;
                  auto hb_yaw = tf2::getYaw(before_pose.pose.orientation);
                  tf2::Quaternion quat;
                  quat.setEuler(segment.twist.twist.angular.z + hb_yaw,0.0,0.0);
                  tf2::convert(before_pose.pose.orientation, quat);
                  tf_->transform(before_pose,after_pose,global_frame, ros::Duration(0.5));
                  agent_pose_vel.velocity.linear.x = after_pose.pose.position.x - agent_pose_vel.pose.position.x;
                  agent_pose_vel.velocity.linear.y = after_pose.pose.position.y - agent_pose_vel.pose.position.y;
                  agent_pose_vel.velocity.angular.z = angles::shortest_angular_distance(tf2::getYaw(after_pose.pose.orientation),tf2::getYaw(agent_pose_vel.pose.orientation));

                  transformed_agents_.push_back(agent_pose_vel);
                }
                catch (tf2::LookupException& ex)
                {
                  ROS_ERROR("No Transform available Error: %s\n", ex.what());
                  continue;
                }
                catch (tf2::ConnectivityException& ex)
                {
                  ROS_ERROR("Connectivity Error: %s\n", ex.what());
                  continue;
                }
                catch (tf2::ExtrapolationException& ex)
                {
                  ROS_ERROR("Extrapolation Error: %s\n", ex.what());
                  continue;
                }
              }
            }
          }
        }
      }
      else if(reset && !shutdown_){
        AgentPoseVel agent_pose_vel;
        agent_pose_vel.header.frame_id = agents_.header.frame_id;
        agent_pose_vel.header.stamp = agents_.header.stamp;
        geometry_msgs::PoseStamped before_pose, after_pose;

        try
        {
          before_pose.pose = segment.pose.pose;
          before_pose.header.frame_id = agents_.header.frame_id;
          before_pose.header.stamp = agents_.header.stamp;
          tf_->transform(before_pose,after_pose,global_frame,ros::Duration(0.5));
          agent_pose_vel.pose = after_pose.pose;

          before_pose.pose.position.x += segment.twist.twist.linear.x;
          before_pose.pose.position.y += segment.twist.twist.linear.y;
          auto hb_yaw = tf2::getYaw(before_pose.pose.orientation);
          tf2::Quaternion quat;
          quat.setEuler(segment.twist.twist.angular.z + hb_yaw,0.0,0.0);
          tf2::convert(before_pose.pose.orientation, quat);
          tf_->transform(before_pose,after_pose,global_frame, ros::Duration(0.5));
          agent_pose_vel.velocity.linear.x = after_pose.pose.position.x - agent_pose_vel.pose.position.x;
          agent_pose_vel.velocity.linear.y = after_pose.pose.position.y - agent_pose_vel.pose.position.y;
          agent_pose_vel.velocity.angular.z = angles::shortest_angular_distance(tf2::getYaw(after_pose.pose.orientation),tf2::getYaw(agent_pose_vel.pose.orientation));

          transformed_agents_.push_back(agent_pose_vel);
        }
        catch (tf2::LookupException& ex)
        {
          ROS_ERROR("No Transform available Error: %s\n", ex.what());
          continue;
        }
        catch (tf2::ConnectivityException& ex)
        {
          ROS_ERROR("Connectivity Error: %s\n", ex.what());
          continue;
        }
        catch (tf2::ExtrapolationException& ex)
        {
          ROS_ERROR("Extrapolation Error: %s\n", ex.what());
          continue;
        }
      }
    }
  }

  updateBoundsFromAgents(min_x, min_y, max_x, max_y);
  if (first_time_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    first_time_ = false;
  }
  else
  {
    double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
    *min_x = std::min(last_min_x_, *min_x);
    *min_y = std::min(last_min_y_, *min_y);
    *max_x = std::max(last_max_x_, *max_x);
    *max_y = std::max(last_max_y_, *max_y);
    last_min_x_ = a;
    last_min_y_ = b;
    last_max_x_ = c;
    last_max_y_ = d;
  }
}
};  // namespace cohan_layers
