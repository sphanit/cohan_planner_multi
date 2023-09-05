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

#include <cohan_layers/static_agent_layer.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <tf2_eigen/tf2_eigen.h>

PLUGINLIB_EXPORT_CLASS(cohan_layers::StaticAgentLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace cohan_layers
{
void StaticAgentLayer::onInitialize()
{
  AgentLayer::onInitialize();
  ros::NodeHandle nh("~/" + name_), g_nh;
  nh.getParam("robot_radius", robot_radius);
  nh.getParam("agent_radius", agent_radius);
  server_ = new dynamic_reconfigure::Server<AgentLayerConfig>(nh);
  f_ = boost::bind(&StaticAgentLayer::configure, this, _1, _2);
  server_->setCallback(f_);
  state_publisher = nh.advertise<std_msgs::String>("person_state", 1);
}

void StaticAgentLayer::updateBoundsFromAgents(double* min_x, double* min_y, double* max_x, double* max_y)
{

  for(uint i=0;i<transformed_agents_.size();i++){
    auto agent = transformed_agents_[i];
    *min_x = std::min(*min_x, agent.pose.position.x - radius_);
    *min_y = std::min(*min_y, agent.pose.position.y - radius_);
    *max_x = std::max(*max_x, agent.pose.position.x + radius_);
    *max_y = std::max(*max_y, agent.pose.position.y + radius_);
}
}

void StaticAgentLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (!enabled_) return;

  if (agents_.agents.size() == 0)
    return;

  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();

  for(uint i=0;i<transformed_agents_.size();i++){
    auto agent = transformed_agents_[i];

    bool check_one=false;
    if(!states_.agents_states.empty()){
      if(((int)states_.agents_states[i].state < 2) && ((int)agents_.agents[i].type == 1)){
        check_one = true;
      }
      else{
        check_one = false;
      }
    }
    else{
      check_one = agents_.agents[i].type;
    }
    
    std_msgs::String state;

    if(check_one){
      std::string temp_1 = "check one true " + std::to_string(agents_.agents[i].track_id);
      state.data = temp_1.c_str();
      unsigned int width = std::max(1, static_cast<int>((2*radius_) / res)),
                   height = std::max(1, static_cast<int>((2*radius_) / res));

      double cx = agent.pose.position.x, cy = agent.pose.position.y;
      double ox = cx - radius_, oy = cy - radius_;

      int mx, my;
      costmap->worldToMapNoBounds(ox, oy, mx, my);

      int start_x = 0, start_y = 0, end_x = width, end_y = height;
      if (mx < 0)
        start_x = -mx;
      else if (mx + width > costmap->getSizeInCellsX())
        end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - mx);

      if (static_cast<int>(start_x + mx) < min_i)
        start_x = min_i - mx;
      if (static_cast<int>(end_x + mx) > max_i)
        end_x = max_i - mx;

      if (my < 0)
        start_y = -my;
      else if (my + height > costmap->getSizeInCellsY())
        end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - my);

      if (static_cast<int>(start_y + my) < min_j)
        start_y = min_j - my;
      if (static_cast<int>(end_y + my) > max_j)
        end_y = max_j - my;

      double bx = ox + res / 2,
             by = oy + res / 2;

      double var = radius_;

      for (int i = start_x; i < end_x; i++)
      {
        for (int j = start_y; j < end_y; j++)
        {
          unsigned char old_cost = costmap->getCost(i + mx, j + my);
          if (old_cost == costmap_2d::NO_INFORMATION)
            continue;

          double x = bx + i * res, y = by + j * res;
          double val;
          val = Gaussian2D(x, y, cx, cy, amplitude_, var, var);
          double rad = sqrt(-2*var*log(val/amplitude_));

          if (rad > radius_)
            continue;
          unsigned char cvalue = (unsigned char) val;
          costmap->setCost(i + mx, j + my, std::max(cvalue, old_cost));
        }
      }
    }

    else{
      std::string temp_2 = "check one false " + std::to_string(agents_.agents[i].track_id);
      state.data = temp_2.c_str();
      double rad;
      if(agents_.agents[i].type==1){
        rad = agent_radius;
      }
      else{
        rad = robot_radius;
      }
      unsigned int width = std::max(1, static_cast<int>((3*rad) / res)),
                   height = std::max(1, static_cast<int>((3*rad) / res));

      double cx = agent.pose.position.x, cy = agent.pose.position.y;
      double ox = cx - rad, oy = cy - rad;

      int mx, my;
      costmap->worldToMapNoBounds(ox, oy, mx, my);

      int start_x = 0, start_y = 0, end_x = width, end_y = height;
      if (mx < 0)
        start_x = -mx;
      else if (mx + width > costmap->getSizeInCellsX())
        end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - mx);

      if (static_cast<int>(start_x + mx) < min_i)
        start_x = min_i - mx;
      if (static_cast<int>(end_x + mx) > max_i)
        end_x = max_i - mx;

      if (my < 0)
        start_y = -my;
      else if (my + height > costmap->getSizeInCellsY())
        end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - my);

      if (static_cast<int>(start_y + my) < min_j)
        start_y = min_j - my;
      if (static_cast<int>(end_y + my) > max_j)
        end_y = max_j - my;

      for (int i = start_x; i < end_x; i++)
      {
        for (int j = start_y; j < end_y; j++)
        {
          unsigned char old_cost = costmap->getCost(i + mx, j + my);
          if (old_cost == costmap_2d::NO_INFORMATION)
            continue;

          double x, y;
          costmap->mapToWorld(i+mx, j+my, x, y);
          x = x-cx, y=y-cy;
          double inrad = sqrt(x*x+y*y);

          if (inrad > rad)
            continue;

          double val;
          if(agents_.agents[i].type==1)
            val = 255;
          else if(agents_.agents[i].type==0)
            val = 255;

          unsigned char cvalue = (unsigned char) val;
          costmap->setCost(i + mx, j + my, std::max(cvalue, old_cost));
        }
      }
    }
    state_publisher.publish(state);
  }
}

void StaticAgentLayer::configure(AgentLayerConfig &config, uint32_t level)
{
  amplitude_ = config.amplitude;
  radius_ = config.radius;
  enabled_ = config.enabled;
}
};  // namespace cohan_layers
