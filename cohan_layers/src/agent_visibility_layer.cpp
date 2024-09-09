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

#include <angles/angles.h>
#include <cohan_layers/agent_visibility_layer.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_eigen/tf2_eigen.h>
#define DEFAULT_AGENT_PART cohan_msgs::TrackedSegmentType::TORSO

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace cohan_layers {
void AgentVisibilityLayer::onInitialize() {
  AgentLayer::onInitialize();
  ros::NodeHandle nh("~/" + name_), g_nh;
  server_ = new dynamic_reconfigure::Server<AgentVLayerConfig>(nh);
  f_ = boost::bind(&AgentVisibilityLayer::configure, this, _1, _2);
  server_->setCallback(f_);
}

void AgentVisibilityLayer::updateBoundsFromAgents(double *min_x, double *min_y,
                                                  double *max_x,
                                                  double *max_y) {

  for (auto &agent : transformed_agents_) {
    *min_x = std::min(*min_x, agent.second.pose.position.x - radius_);
    *min_y = std::min(*min_y, agent.second.pose.position.y - radius_);
    *max_x = std::max(*max_x, agent.second.pose.position.x + radius_);
    *max_y = std::max(*max_y, agent.second.pose.position.y + radius_);
  }
}

void AgentVisibilityLayer::updateCosts(costmap_2d::Costmap2D &master_grid,
                                       int min_i, int min_j, int max_i,
                                       int max_j) {
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (!enabled_)
    return;

  if (agents_.agents.size() == 0)
    return;

  costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();

  auto it = transformed_agents_.begin();
  for (uint i = 0; i < transformed_agents_.size(); i++) {
    auto &agent = it->second;
    double theta = tf2::getYaw(agent.pose.orientation);
    Eigen::Vector2d orient_vec(std::cos(theta), std::sin(theta));

    if ((int)agents_.agents[it->first - 1].type == 1) {
      if (!states_.states.empty()) {
        if ((int)states_.states[it->first - 1] > 1)
          continue;
      }
      unsigned int width = std::max(1, static_cast<int>((2 * radius_) / res)),
                   height = std::max(1, static_cast<int>((2 * radius_) / res));

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

      double bx = ox + res / 2, by = oy + res / 2;

      double var = radius_;

      for (int i = start_x; i < end_x; i++) {
        for (int j = start_y; j < end_y; j++) {
          unsigned char old_cost = costmap->getCost(i + mx, j + my);
          if (old_cost == costmap_2d::NO_INFORMATION)
            continue;

          double x = bx + i * res, y = by + j * res;
          double val;

          val = Gaussian2D(x, y, cx, cy, amplitude_, var, var);

          double rad = sqrt(-2 * var * log(val / amplitude_));
          Eigen::Vector2d pt_vec(x - cx, y - cy);

          if (rad > radius_)
            continue;
          unsigned char cvalue = (unsigned char)val; // std::min(5*val,254.0);
          if (orient_vec.dot(pt_vec) <= 0)
            costmap->setCost(i + mx, j + my, std::max(cvalue, old_cost));
        }
      }
    }
  }
}

void AgentVisibilityLayer::configure(AgentVLayerConfig &config,
                                     uint32_t level) {
  amplitude_ = config.amplitude;
  radius_ = config.radius;
  enabled_ = config.enabled;
}
}; // namespace cohan_layers

PLUGINLIB_EXPORT_CLASS(cohan_layers::AgentVisibilityLayer, costmap_2d::Layer)
