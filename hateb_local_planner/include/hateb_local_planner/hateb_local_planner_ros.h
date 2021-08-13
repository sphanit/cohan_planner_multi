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
 * Author: Christoph Rösmann
 * Modified by: Phani Teja Singamaneni
 *********************************************************************/

#ifndef HATEB_LOCAL_PLANNER_ROS_H_
#define HATEB_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>


// timed-elastic-band related classes
#include <hateb_local_planner/optimal_planner.h>
#include <hateb_local_planner/homotopy_class_planner.h>
#include <hateb_local_planner/visualization.h>
#include <hateb_local_planner/recovery_behaviors.h>

// message types
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <costmap_converter/ObstacleMsg.h>
#include <hateb_local_planner/Optimize.h>
#include <hateb_local_planner/Approach.h>
#include <hateb_local_planner/setPlanning.h>
#include <hateb_local_planner/getPlanning.h>
#include <hateb_local_planner/TransformedPlan.h>

// agent data
#include <agent_path_prediction/AgentPosePredict.h>
#include <agent_path_prediction/agent_path_prediction.h>
#include <agent_path_prediction/PredictedGoal.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>
#include <cohan_msgs/StateArray.h>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/impl/utils.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_converter/costmap_converter_interface.h>


// dynamic reconfigure
#include <hateb_local_planner/HATebLocalPlannerReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>


// Backoff recovery
#include <hateb_local_planner/backoff.h>


namespace hateb_local_planner
{
enum AgentState {NO_STATE,STATIC,MOVING,STOPPED,BLOCKED};

/**
  * @class HATebLocalPlannerROS
  * @brief Implements both nav_core::BaseLocalPlanner and mbf_costmap_core::CostmapController abstract
  * interfaces, so the hateb_local_planner plugin can be used both in move_base and move_base_flex (MBF).
  * @todo Escape behavior, more efficient obstacle handling
  */
class HATebLocalPlannerROS : public nav_core::BaseLocalPlanner, public mbf_costmap_core::CostmapController
{

public:
  /**
    * @brief Default constructor of the teb plugin
    */
  HATebLocalPlannerROS();

  /**
    * @brief  Destructor of the plugin
    */
  ~HATebLocalPlannerROS();

  /**
    * @brief Initializes the teb plugin
    * @param name The name of the instance
    * @param tf Pointer to a tf buffer
    * @param costmap_ros Cost map representing occupied and free space
    */
  void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
    * @brief Set the plan that the teb local planner is following
    * @param orig_global_plan The plan to pass to the local planner
    * @return True if the plan was updated successfully, false otherwise
    */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
    * @return True if a valid trajectory was found, false otherwise
    */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base.
    * @remark Extended version for MBF API
    * @param pose the current pose of the robot.
    * @param velocity the current velocity of the robot.
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
    * @param message Optional more detailed outcome as a string
    * @return Result code as described on ExePath action result:
    *         SUCCESS         = 0
    *         1..9 are reserved as plugin specific non-error results
    *         FAILURE         = 100   Unspecified failure, only used for old, non-mfb_core based plugins
    *         CANCELED        = 101
    *         NO_VALID_CMD    = 102
    *         PAT_EXCEEDED    = 103
    *         COLLISION       = 104
    *         OSCILLATION     = 105
    *         ROBOT_STUCK     = 106
    *         MISSED_GOAL     = 107
    *         MISSED_PATH     = 108
    *         BLOCKED_PATH    = 109
    *         INVALID_PATH    = 110
    *         TF_ERROR        = 111
    *         NOT_INITIALIZED = 112
    *         INVALID_PLUGIN  = 113
    *         INTERNAL_ERROR  = 114
    *         121..149 are reserved as plugin specific errors
    */
  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity,
                                   geometry_msgs::TwistStamped &cmd_vel, std::string &message);


  /**
    * @brief  Check if the goal pose has been achieved
    *
    * The actual check is performed in computeVelocityCommands().
    * Only the status flag is checked here.
    * @return True if achieved, false otherwise
    */
  bool isGoalReached();


  /**

    * @brief Dummy version to satisfy MBF API
    */
  bool isGoalReached(double xy_tolerance, double yaw_tolerance) { return isGoalReached(); };

  /**
    * @brief Requests the planner to cancel, e.g. if it takes too much time
    * @remark New on MBF API
    * @return True if a cancel has been successfully requested, false if not implemented.
    */
  bool cancel() { return false; };




  /** @name Public utility functions/methods */
  //@{

    /**
    * @brief  Transform a tf::Pose type into a Eigen::Vector2d containing the translational and angular velocities.
    *
    * Translational velocities (x- and y-coordinates) are combined into a single translational velocity (first component).
    * @param tf_vel tf::Pose message containing a 1D or 2D translational velocity (x,y) and an angular velocity (yaw-angle)
    * @return Translational and angular velocity combined into an Eigen::Vector2d
    */
  static Eigen::Vector2d tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel);

  /**
   * @brief Get the current robot footprint/contour model
   * @param nh const reference to the local ros::NodeHandle
   * @return Robot footprint model used for optimization
   */
  static RobotFootprintModelPtr getRobotFootprintFromParamServer(const ros::NodeHandle& nh);

    /**
   * @brief Set the footprint from the given XmlRpcValue.
   * @remarks This method is copied from costmap_2d/footprint.h, since it is not declared public in all ros distros
   * @remarks It is modified in order to return a container of Eigen::Vector2d instead of geometry_msgs::Point
   * @param footprint_xmlrpc should be an array of arrays, where the top-level array should have 3 or more elements, and the
   * sub-arrays should all have exactly 2 elements (x and y coordinates).
   * @param full_param_name this is the full name of the rosparam from which the footprint_xmlrpc value came.
   * It is used only for reporting errors.
   * @return container of vertices describing the polygon
   */
  static Point2dContainer makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name);

  /**
   * @brief Get a number from the given XmlRpcValue.
   * @remarks This method is copied from costmap_2d/footprint.h, since it is not declared public in all ros distros
   * @remarks It is modified in order to return a container of Eigen::Vector2d instead of geometry_msgs::Point
   * @param value double value type
   * @param full_param_name this is the full name of the rosparam from which the footprint_xmlrpc value came.
   * It is used only for reporting errors.
   * @returns double value
   */
  static double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);

  //@}

protected:

  /**
    * @brief Update internal obstacle vector based on occupied costmap cells
    * @remarks All occupied cells will be added as point obstacles.
    * @remarks All previous obstacles are cleared.
    * @sa updateObstacleContainerWithCostmapConverter
    * @todo Include temporal coherence among obstacle msgs (id vector)
    * @todo Include properties for dynamic obstacles (e.g. using constant velocity model)
    */
  void updateObstacleContainerWithCostmap();

  /**
   * @brief Update internal obstacle vector based on polygons provided by a costmap_converter plugin
   * @remarks Requires a loaded costmap_converter plugin.
   * @remarks All previous obstacles are cleared.
   * @sa updateObstacleContainerWithCostmap
   */
  void updateObstacleContainerWithCostmapConverter();

  /**
   * @brief Update internal obstacle vector based on custom messages received via subscriber
   * @remarks All previous obstacles are NOT cleared. Call this method after other update methods.
   * @sa updateObstacleContainerWithCostmap, updateObstacleContainerWithCostmapConverter
   */
  void updateObstacleContainerWithCustomObstacles();


  /**
   * @brief Update internal via-point container based on the current reference plan
   * @remarks All previous via-points will be cleared.
   * @param transformed_plan (local) portion of the global plan (which is already transformed to the planning frame)
   * @param min_separation minimum separation between two consecutive via-points
   */
  void updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation);

  void updateAgentViaPointsContainers(
      const AgentPlanVelMap &transformed_agent_plan_vel_map,
      double min_separation);

  /**
    * @brief Callback for the dynamic_reconfigure node.
    *
    * This callback allows to modify parameters dynamically at runtime without
    *restarting the node
    * @param config Reference to the dynamic reconfigure config
    * @param level Dynamic reconfigure level
    */
  void reconfigureCB(HATebLocalPlannerReconfigureConfig& config, uint32_t level);
  /**
   * @brief Callback for custom obstacles that are not obtained from the costmap
   * @param obst_msg pointer to the message containing a list of polygon shaped
   * obstacles
   */
  void customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);

   /**
    * @brief Callback for custom via-points
    * @param via_points_msg pointer to the message containing a list of via-points
    */
  void customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg);

   /**
    * @brief Prune global plan such that already passed poses are cut off
    *
    * The pose of the robot is transformed into the frame of the global plan by taking the most recent tf transform.
    * If no valid transformation can be found, the method returns \c false.
    * The global plan is pruned until the distance to the robot is at least \c dist_behind_robot.
    * If no pose within the specified treshold \c dist_behind_robot can be found,
    * nothing will be pruned and the method returns \c false.
    * @remarks Do not choose \c dist_behind_robot too small (not smaller the cellsize of the map), otherwise nothing will be pruned.
    * @param tf A reference to a tf buffer
    * @param global_pose The global pose of the robot
    * @param[in,out] global_plan The plan to be transformed
    * @param dist_behind_robot Distance behind the robot that should be kept [meters]
    * @return \c true if the plan is pruned, \c false in case of a transform exception or if no pose cannot be found inside the threshold
    */
  bool pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                       std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot=1);

  /**
    * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
    *
    * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h
    * such that the index of the current goal pose is returned as well as
    * the transformation between the global plan and the planning frame.
    * @param tf A reference to a tf buffer
    * @param global_plan The plan to be transformed
    * @param global_pose The global pose of the robot
    * @param costmap A reference to the costmap being used so the window size for transforming can be computed
    * @param global_frame The frame to transform the plan to
    * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0: disabled; the length is also bounded by the local costmap size!]
    * @param[out] transformed_plan Populated with the transformed plan
    * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
    * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
    * @return \c true if the global plan is transformed, \c false otherwise
    */
  bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                           const geometry_msgs::PoseStamped& global_pose,  const costmap_2d::Costmap2D& costmap,
                           const std::string& global_frame, double max_plan_length, PlanCombined &transformed_plan_combined,
                           int* current_goal_idx = NULL, geometry_msgs::TransformStamped* tf_plan_to_global = NULL) const;

  /**
    * @brief  Transforms the agent plan from the tracker frame to the local
   * frame.
    *
    * @param tf A reference to a transform listener
    * @param agent_plan The plan to be transformed
    * @param global_pose The global pose of the robot
    * @param costmap A reference to the costmap being used so the window size
   * for transforming can be computed
    * @param global_frame The frame to transform the plan to
    * @param[out] transformed_agent_plan Populated with the transformed plan
    * @param[out] tf_agent_plan_to_global Transformation between the agent plan
   * and the local planning frame
    * @return \c true if the global plan is transformed, \c false otherwise
    */
  bool transformAgentPlan(
      const tf2_ros::Buffer &tf2, const geometry_msgs::PoseStamped &robot_pose,
      const costmap_2d::Costmap2D &costmap, const std::string &global_frame,
      const std::vector<geometry_msgs::PoseWithCovarianceStamped> &agent_plan,
      AgentPlanCombined &transformed_agent_plan_combined,
      geometry_msgs::TwistStamped &transformed_agent_twist,
      tf2::Stamped<tf2::Transform> *tf_agent_plan_to_global = NULL) const;

  bool transformAgentPose(const tf2_ros::Buffer &tf2,
                     const std::string &global_frame,
                     geometry_msgs::PoseWithCovarianceStamped &agent_pose,
                     geometry_msgs::PoseStamped &transformed_agent_pose) const;

  /**
    * @brief Estimate the orientation of a pose from the global_plan that is treated as a local goal for the local planner.
    *
    * If the current (local) goal point is not the final one (global)
    * substitute the goal orientation by the angle of the direction vector between
    * the local goal and the subsequent pose of the global plan.
    * This is often helpful, if the global planner does not consider orientations. \n
    * A moving average filter is utilized to smooth the orientation.
    * @param global_plan The global plan
    * @param local_goal Current local goal
    * @param current_goal_idx Index of the current (local) goal pose in the global plan
    * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
    * @param moving_average_length number of future poses of the global plan to be taken into account
    * @return orientation (yaw-angle) estimate
    */
  double estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
                                      int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length=3) const;


  /**
   * @brief Saturate the translational and angular velocity to given limits.
   *
   * The limit of the translational velocity for backwards driving can be changed independently.
   * Do not choose max_vel_x_backwards <= 0. If no backward driving is desired, change the optimization weight for
   * penalizing backwards driving instead.
   * @param[in,out] vx The translational velocity that should be saturated.
   * @param[in,out] vy Strafing velocity which can be nonzero for holonomic robots
   * @param[in,out] omega The angular velocity that should be saturated.
   * @param max_vel_x Maximum translational velocity for forward driving
   * @param max_vel_y Maximum strafing velocity (for holonomic robots)
   * @param max_vel_theta Maximum (absolute) angular velocity
   * @param max_vel_x_backwards Maximum translational velocity for backwards driving
   */
  void saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y,
                        double max_vel_theta, double max_vel_x_backwards);


  /**
   * @brief Convert translational and rotational velocities to a steering angle of a carlike robot
   *
   * The conversion is based on the following equations:
   * - The turning radius is defined by \f$ R = v/omega \f$
   * - For a car like robot withe a distance L between both axles, the relation is: \f$ tan(\phi) = L/R \f$
   * - phi denotes the steering angle.
   * @remarks You might provide distances instead of velocities, since the temporal information is not required.
   * @param v translational velocity [m/s]
   * @param omega rotational velocity [rad/s]
   * @param wheelbase distance between both axles (drive shaft and steering axle), the value might be negative for back_wheeled robots
   * @param min_turning_radius Specify a lower bound on the turning radius
   * @return Resulting steering angle in [rad] inbetween [-pi/2, pi/2]
   */
  double convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius = 0) const;

  /**
   * @brief Validate current parameter values of the footprint for optimization, obstacle distance and the costmap footprint
   *
   * This method prints warnings if validation fails.
   * @remarks Currently, we only validate the inscribed radius of the footprints
   * @param opt_inscribed_radius Inscribed radius of the RobotFootprintModel for optimization
   * @param costmap_inscribed_radius Inscribed radius of the footprint model used for the costmap
   * @param min_obst_dist desired distance to obstacles
   */
  void configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx);

  void validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist);

  // Agent Prediction reset
  void resetAgentsPrediction();

  // Tracked agents call back to integrate agents into planning
  void agentsCB(const cohan_msgs::TrackedAgents &tracked_agents);


  // Standalone optimization for analysis
  bool optimizeStandalone(hateb_local_planner::Optimize::Request &req,
                          hateb_local_planner::Optimize::Response &res);
  // Apporach Mode
  bool setApproachID(hateb_local_planner::Approach::Request &req,
                     hateb_local_planner::Approach::Response &res);

  bool setPlanningMode(hateb_local_planner::setPlanning::Request &req,
                       hateb_local_planner::setPlanning::Response &res);

  bool getPlanningMode(hateb_local_planner::getPlanning::Request &req,
                       hateb_local_planner::getPlanning::Response &res);



  int getLatestCommonTime(const std::string &source_frame, const std::string &target_frame, ros::Time& time, std::string* error_string) const;

  void lookupTwist(const std::string& tracking_frame, const std::string& observation_frame,
                                const ros::Time& time, const ros::Duration& averaging_interval,
                                geometry_msgs::Twist& twist) const;

  void lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
                   const tf2::Vector3 & reference_point, const std::string& reference_point_frame,
                   const ros::Time& time, const ros::Duration& averaging_interval,
                   geometry_msgs::Twist& twist) const;

   // bool static isEqual(const std::pair<std::string, int>& element){return element.first ==  User.name;}

private:
  // Definition of member variables

  // external objects (store weak pointers)
  costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
  costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
  tf2_ros::Buffer* tf_; //!< pointer to tf buffer

  // internal objects (memory management owned)
  PlannerInterfacePtr planner_; //!< Instance of the underlying optimal planner class
  ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
  ViaPointContainer via_points_; //!< Container of via-points that should be considered during local trajectory optimization
  std::map<uint64_t, ViaPointContainer> agents_via_points_map_;
  TebVisualizationPtr visualization_; //!< Instance of the visualization class (local/global plan, obstacles, ...)
  boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;
  HATebConfig cfg_; //!< Config class that stores and manages all related parameters
  HATebLocalPlannerReconfigureConfig config_;
  FailureDetector failure_detector_; //!< Detect if the robot got stucked
  Backoff backoff_recovery_;

  std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan

  base_local_planner::OdometryHelperRos odom_helper_; //!< Provides an interface to receive the current velocity from the robot

  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime
  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_; //!< Store the current costmap_converter

  boost::shared_ptr< dynamic_reconfigure::Server<HATebLocalPlannerReconfigureConfig> > dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
  ros::Subscriber custom_obst_sub_; //!< Subscriber for custom obstacles received via a ObstacleMsg.
  boost::mutex custom_obst_mutex_; //!< Mutex that locks the obstacle array (multi-threaded)
  costmap_converter::ObstacleArrayMsg custom_obstacle_msg_; //!< Copy of the most recent obstacle message

  ros::Subscriber via_points_sub_; //!< Subscriber for custom via-points received via a Path msg.
  bool custom_via_points_active_; //!< Keep track whether valid via-points have been received from via_points_sub_
  boost::mutex via_point_mutex_; //!< Mutex that locks the via_points container (multi-threaded)

  PoseSE2 robot_pose_; //!< Store current robot pose
  PoseSE2 robot_goal_; //!< Store current robot goal
  geometry_msgs::Twist robot_vel_; //!< Store current robot translational and angular velocity (vx, vy, omega)
  bool goal_reached_; //!< store whether the goal is reached or not
  bool horizon_reduced_;
  ros::Time horizon_reduced_stamp_;
  ros::Time time_last_infeasible_plan_; //!< Store at which time stamp the last infeasible plan was detected
  int no_infeasible_plans_; //!< Store how many times in a row the planner failed to find a feasible plan.
  ros::Time time_last_oscillation_; //!< Store at which time stamp the last oscillation was detected
  RotType last_preferred_rotdir_; //!< Store recent preferred turning direction
  geometry_msgs::Twist last_cmd_; //!< Store the last control command generated in computeVelocityCommands()

  std::vector<geometry_msgs::Point> footprint_spec_; //!< Store the footprint of the robot
  double robot_inscribed_radius_; //!< The radius of the inscribed circle of the robot (collision possible)
  double robot_circumscribed_radius; //!< The radius of the circumscribed circle of the robot

  std::string global_frame_; //!< The frame in which the controller will run
  std::string robot_base_frame_; //!< Used as the base frame id of the robot

  // flags
  bool initialized_; //!< Keeps track about the correct initialization of this
                     //!class

  // agent perdiction service
  ros::ServiceClient predict_agents_client_, reset_agents_prediction_client_,
      publish_predicted_markers_client_, agent_goal_client_;

  std::string predict_srv_name_, reset_prediction_srv_name_, publish_makers_srv_name_, agent_goal_srv_name_;

  // optimize service
  ros::ServiceServer optimize_server_, approach_server_, planning_server_, get_plan_server_;

  /* Agents in the loop */
  // Agent prediction services
  bool publish_predicted_agent_markers_ = true;
  ros::Time last_call_time_;

  // Tracked agents in HATeb
  cohan_msgs::TrackedAgents tracked_agents_,prev_tracked_agents_; //Tracked agents from an external module
  geometry_msgs::Pose robot_pos_msg, last_robot_pose; //Robot pose messages for context analysis

  //Planner State determining parameters
  // different flags, times and measures
  bool isDistunderThreshold, isDistMax, stuck, goal_ctrl, reset_states, ext_goal, backed_off;
  std::vector<bool> agent_still;
  ros::Time last_position_time, last_omega_sign_change_;
  double last_omega_;

  // Mode of the Planner (or planning state holder) and counter
  int isMode, change_mode;

  // Agent ids, states and velocities
  std::vector<int> visible_agent_ids; // List of visible agents
  std::vector<std::vector<double>> agent_vels; // List of agent velocities over time
  std::vector<double> agent_nominal_vels; // Nominal velocities  of agents based on moving average filter
  cohan_msgs::StateArray agents_states_; // State of agents

  // Backoff recovery params
  int stuck_agent_id; // Stores the agent id who blocked the robot's way during backoff recovery
  double ang_theta; // Re-orientation angle

  // Safety costmap for agents
  std::map<uint64_t, std::vector<geometry_msgs::Point>> agent_prev_pos_costmap;

  // Logs of the system
  std::string logs;
  double current_agent_dist;

  // Tracked agents Subscriber
  ros::Subscriber agents_sub_;

  // Logs and agent states publishers
  ros::Publisher log_pub_, agents_states_pub_, transformed_global_plan_pub_;

  //Name space to support multiple agents
  std::string ns_;

  // Optional publishers
  // ros::Publisher op_costs_pub_, robot_pose_pub_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}; // end namespace hateb_local_planner

#endif // HATEB_LOCAL_PLANNER_ROS_H_
