/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
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
 * Author: Phani Teja Singamaneni
 *********************************************************************/
 #ifndef OPTIMIZED_VEL_H_
 #define OPTIMIZED_VEL_H_

 #include <ros/ros.h>

 #include <tf2_ros/transform_listener.h>
 #include <tf/tf.h>
 #include <tf2/utils.h>
 #include <tf2/impl/utils.h>
 #include <tf2/convert.h>
 #include <tf2_ros/buffer.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 #include <tf/transform_listener.h>

 #include <nav_msgs/GetPlan.h>
 #include <nav_msgs/Path.h>
 #include <cohan_msgs/TrackedAgents.h>
 #include <cohan_msgs/AgentPathArray.h>
 #include <cohan_msgs/AgentPath.h>
 #include <cohan_msgs/AgentTrajectoryArray.h>
 #include <cohan_msgs/TrackedSegmentType.h>

 #include <hateb_local_planner/Optimize.h>
 #include <hateb_local_planner/getOptimVel.h>


namespace hateb_local_planner{
   class OptimizedVel{
   public:
     OptimizedVel(tf2_ros::Buffer &tf2_);
     ~OptimizedVel();
     void initialize();
     void UpdateStartPoses(const cohan_msgs::TrackedAgents &tracked_agents);
     void correctPose(geometry_msgs::Pose &behind_pose);
     bool checkGoal(geometry_msgs::PoseStamped goal);
     geometry_msgs::Twist OptimizeAndgetVel(const geometry_msgs::PoseStamped &robot_goal);
     bool get_vel_srv(hateb_local_planner::getOptimVel::Request &req, hateb_local_planner::getOptimVel::Response &res);

   private:
     tf2_ros::Buffer &tf_;
     tf2_ros::TransformListener tfListener_;
     bool initialized_, predict_behind_robot_, got_robot_plan, got_agent_plan;
     ros::ServiceClient optimize_client, getPlan_client;
     ros::Subscriber agents_sub_, robot_goal_sub_;
     ros::ServiceServer get_vel_srv_;

     std::vector<geometry_msgs::PoseStamped> agents_start_poses, agents_goals_;
     geometry_msgs::PoseStamped robot_start_pose, robot_goal_;
     geometry_msgs::TransformStamped robot_to_map_tf;
     cohan_msgs::TrackedAgents tracked_agents_;
     cohan_msgs::AgentPathArray agents_plans;
     nav_msgs::Path robot_plan;

 };// class OptimizedVel
};// namespace hateb_local_planner
#endif // OPTIMIZED_VEL_H_
