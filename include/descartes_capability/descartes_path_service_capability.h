/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik LLC
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *********************************************************************/

/* Author: Mike Lautman - mike@picknik.ai
   Description: MoveIt! capability for planning Cartesian paths that uses descartes
   Note: This is different from a normal capability in that it expects certain
         rosparams to be on the server in order to funciton properly.
 */

#ifndef MOVEIT_MOVE_GROUP_DESCARTES_PATH_SERVICE_CAPABILITY_
#define MOVEIT_MOVE_GROUP_DESCARTES_PATH_SERVICE_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/GetCartesianPath.h>

// Descartes
#include <descartes_core/robot_model.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>

// Eigen
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace descartes_capability
{
class MoveGroupDescartesPathService : public move_group::MoveGroupCapability
{
public:
  MoveGroupDescartesPathService();

  virtual void initialize();

private:
  bool computeService(moveit_msgs::GetCartesianPath::Request& req, moveit_msgs::GetCartesianPath::Response& res);

  /** \brief Initializes descartes_model_ with new parameters **/
  bool initializeDescartesModel(const std::string& group_name, const std::string& world_frame,
                                const std::string& link_name);

  /** \brief Computes the maximum joint delta between two states. Returns -1.0 if vector size mismatch **/
  double computeMaxJointDelta(const std::vector<double>& joints1, const std::vector<double>& joints2);

  /** \brief Interpolates between start and end poses and appends them to deense_waypoints **/
  static void createDensePath(const Eigen::Isometry3d& start, const Eigen::Isometry3d& end, double max_step,
                              EigenSTL::vector_Isometry3d& dense_waypoints);

  /** \brief Transforms each point in a vector of affine**/
  void createDescartesTrajectory(const EigenSTL::vector_Isometry3d& dense_waypoints,
                                 std::vector<descartes_core::TrajectoryPtPtr>& input_descartes_trajectory);

  /** \brief Transforms each waypoint in the request to the target frame **/
  bool transformWaypointsToFrame(const moveit_msgs::GetCartesianPath::Request& req, const std::string& target_frame,
                                 EigenSTL::vector_Isometry3d& waypoints);

  /** \brief Takes in a trajectory computed by Descartes and populates the robot_trajectory**/
  double copyDescartesResultToRobotTrajectory(const std::vector<descartes_core::TrajectoryPtPtr>& descartes_result,
                                              const moveit_msgs::GetCartesianPath::Request& req,
                                              robot_trajectory::RobotTrajectory& robot_trajectory);

  void printDelta(const std::vector<double>& joints1, const std::vector<double>& joints2);
  void printJoints(const std::vector<double>& joints);
  void printJointsNamed(const std::string& name, const std::vector<double>& joints);

  ros::NodeHandle nh_;
  std::string name_ = "descartes_path_service_capability";

  // For setting up and generating Cartesian trajectories with Descartes
  descartes_core::RobotModelPtr descartes_model_;

  // Params loaded from server
  double positional_tolerance_;
  double positional_tolerance_increment_;
  double roll_orientation_tolerance_;
  double pitch_orientation_tolerance_;
  double yaw_orientation_tolerance_;
  double orientation_tolerance_increment_;

  bool verbose_debug_;
  bool visual_debug_;

  ros::ServiceServer descartes_path_service_;
  bool display_computed_paths_;

  // Cached values for checking if we need to re-initialize our descartes_model
  std::string current_group_name_;
  std::string current_world_frame_;
  std::string current_tcp_frame_;

  // For Rviz visualizations
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
};
}  // namespace descartes_capability

#endif  // MOVEIT_MOVE_GROUP_DESCARTES_PATH_SERVICE_CAPABILITY_
