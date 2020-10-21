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

#include <descartes_capability/descartes_path_service_capability.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>

namespace descartes_capability
{
MoveGroupDescartesPathService::MoveGroupDescartesPathService()
  : MoveGroupCapability("DescartesPathService")
  , nh_("~")
  , positional_tolerance_(0.0)
  , positional_tolerance_increment_(0.0)
  , roll_orientation_tolerance_(0.0)
  , pitch_orientation_tolerance_(0.0)
  , yaw_orientation_tolerance_(0.0)
  , orientation_tolerance_increment_(0.0)
  , verbose_debug_(false)
  , visual_debug_(false)
  , display_computed_paths_(true)
{
}

void MoveGroupDescartesPathService::initialize()
{
  // Get parameters from param server
  const std::string node_name = "MoveGroupDescartesPathService";
  ros::NodeHandle rosparam_nh(nh_, node_name);

  nh_.param<bool>("descartes_params/debug/verbose", verbose_debug_, false);
  nh_.param<bool>("descartes_params/debug/visual", visual_debug_, false);

  context_->planning_scene_monitor_->updateFrameTransforms();

  // For visualizing the path request
  const std::string& world_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(world_frame, "/rviz_visual_tools"));
  if (visual_debug_)
  {
    visual_tools_->loadMarkerPub(true);
    visual_tools_->deleteAllMarkers();
  }

  // For displaying the computed trajectory
  visual_tools_->loadSharedRobotState();
  visual_tools_->loadTrajectoryPub(planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC, false);

  descartes_path_service_ = root_node_handle_.advertiseService(move_group::CARTESIAN_PATH_SERVICE_NAME,
                                                               &MoveGroupDescartesPathService::computeService, this);
}

void MoveGroupDescartesPathService::createDensePath(const Eigen::Isometry3d& start, const Eigen::Isometry3d& end,
                                                    double max_step, EigenSTL::vector_Isometry3d& dense_waypoints)
{
  // TODO: There's a lot of casting being done. This could probably be pruned down
  const Eigen::Quaterniond start_quaternion(start.rotation());
  const Eigen::Quaterniond end_quaternion(end.rotation());

  const Eigen::Vector3d start_translation = start.translation();
  const Eigen::Vector3d end_translation = end.translation();

  const double distance = (start_translation - end_translation).norm();
  const double step_size = max_step / distance;

  // Start from the second point and go all the way to the end. This is important
  // for multiple waypoint trajectories to ensure no duplicate points and that the
  // path ends at the last waypoint
  for (double t = step_size; t < 1.0 + step_size; t += step_size)
  {
    if (t > 1.0)
      t = 1.0;  // Ensure last step is at exactly 1.0

    Eigen::Isometry3d eigen_pose;
    eigen_pose.translation() = (end_translation - start_translation) * t + start_translation;
    eigen_pose.linear() = start_quaternion.slerp(t, end_quaternion).toRotationMatrix();

    dense_waypoints.push_back(eigen_pose);
  }
}

void MoveGroupDescartesPathService::createDescartesTrajectory(
    const EigenSTL::vector_Isometry3d& dense_waypoints,
    std::vector<descartes_core::TrajectoryPtPtr>& input_descartes_trajectory)
{
  for (auto eigen_pose : dense_waypoints)
  {
    const Eigen::Quaterniond rotation(eigen_pose.rotation());

    if (verbose_debug_)
      ROS_DEBUG_NAMED(name_, "x: %.3f\ty: %.3f\tz: %.3f\twx: %.3f\twy: %.3f\twz: %.3f\tww: %.3f\t",
                      eigen_pose.translation().x(), eigen_pose.translation().y(), eigen_pose.translation().z(),
                      rotation.x(), rotation.y(), rotation.z(), rotation.w());

    if (positional_tolerance_ == 0.0 && roll_orientation_tolerance_ == 0.0 && pitch_orientation_tolerance_ == 0 &&
        yaw_orientation_tolerance_ == 0)
    {
      const descartes_core::TrajectoryPtPtr descartes_point = descartes_core::TrajectoryPtPtr(
          new descartes_trajectory::CartTrajectoryPt(descartes_trajectory::TolerancedFrame(eigen_pose)));
      input_descartes_trajectory.push_back(descartes_point);
    }
    else
    {
      descartes_trajectory::PositionTolerance position_tol =
          descartes_trajectory::ToleranceBase::createSymmetric<descartes_trajectory::PositionTolerance>(
              eigen_pose.translation().x(), eigen_pose.translation().y(), eigen_pose.translation().z(),
              positional_tolerance_);

      Eigen::Matrix3d m = Eigen::Matrix3d(rotation);
      Eigen::Vector3d rxyz = m.eulerAngles(0, 1, 2);

      descartes_trajectory::OrientationTolerance orientation_tol =
          descartes_trajectory::ToleranceBase::createSymmetric<descartes_trajectory::OrientationTolerance>(
              rxyz(0), rxyz(1), rxyz(2), roll_orientation_tolerance_, pitch_orientation_tolerance_,
              yaw_orientation_tolerance_);

      descartes_core::TrajectoryPtPtr descartes_point =
          descartes_core::TrajectoryPtPtr(new descartes_trajectory::CartTrajectoryPt(
              descartes_trajectory::TolerancedFrame(eigen_pose, position_tol, orientation_tol),
              positional_tolerance_increment_, orientation_tolerance_increment_));
      input_descartes_trajectory.push_back(descartes_point);
    }
  }
}

double MoveGroupDescartesPathService::computeMaxJointDelta(const std::vector<double>& joints1,
                                                           const std::vector<double>& joints2)
{
  if (joints1.size() != joints2.size())
  {
    ROS_ERROR_NAMED(name_, "computeMaxJointDelta received vectors of mismatched size");
    return -1.0;
  }
  double max_delta = 0.0;
  for (std::size_t ix = 0; ix < joints1.size(); ++ix)
  {
    max_delta = std::max(max_delta, std::abs(joints1[ix] - joints2[ix]));
  }
  return max_delta;
}

bool MoveGroupDescartesPathService::transformWaypointsToFrame(const moveit_msgs::GetCartesianPath::Request& req,
                                                              const std::string& target_frame,
                                                              EigenSTL::vector_Isometry3d& waypoints)
{
  for (std::size_t i = 0; i < req.waypoints.size(); ++i)
  {
    geometry_msgs::PoseStamped p;
    p.header = req.header;
    p.pose = req.waypoints[i];
    if (performTransform(p, target_frame))
      tf::poseMsgToEigen(p.pose, waypoints[i]);
    else
      return false;
  }
  return true;
}

double MoveGroupDescartesPathService::copyDescartesResultToRobotTrajectory(
    const std::vector<descartes_core::TrajectoryPtPtr>& descartes_result,
    const moveit_msgs::GetCartesianPath::Request& req, robot_trajectory::RobotTrajectory& robot_trajectory)
{
  if (verbose_debug_)
    ROS_DEBUG_NAMED(name_, "Full trajectory of length %zu", descartes_result.size());

  std::vector<double> next_positions;
  std::vector<double> last_positions;
  bool first_point = true;
  bool joint_threshold_exceeded = false;
  double fraction = 1.0;

  robot_state::RobotState start_state =
      planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();

  robot_state::robotStateMsgToRobotState(req.start_state, start_state);

  const robot_model::JointModelGroup* jmg;
  jmg = start_state.getJointModelGroup(req.group_name);
  for (std::size_t i = 0; i < descartes_result.size() && !joint_threshold_exceeded; ++i)
  {
    descartes_result[i]->getNominalJointPose({}, *descartes_model_, next_positions);

    if (verbose_debug_)
      printJoints(next_positions);

    double max_delta = 0.0;
    if (!first_point)
      max_delta = computeMaxJointDelta(next_positions, last_positions);

    if (req.jump_threshold > 0.0 && max_delta > req.jump_threshold)
    {
      ROS_WARN_NAMED(name_, "Jump threshold of %.3f exceeded with requested jump of %.3f", req.jump_threshold,
                     max_delta);
      fraction = (double)i / (double)descartes_result.size();
      joint_threshold_exceeded = true;
    }
    else if (!first_point && max_delta <= 0.00001 && req.jump_threshold > 0.0)
    {
      ROS_WARN_NAMED(name_, "Duplicate point found with max delta of %.6f", max_delta);
    }
    else
    {
      start_state.setJointGroupPositions(jmg, next_positions);
      // Append joint values to robot_trajectory
      double delta_time = 0.00001;
      robot_trajectory.addSuffixWayPoint(start_state, delta_time);
      last_positions = next_positions;
    }
    first_point = false;
  }

  return fraction;
}

bool MoveGroupDescartesPathService::initializeDescartesModel(const std::string& group_name,
                                                             const std::string& world_frame,
                                                             const std::string& tcp_frame)
{
  // Setup Descartes model
  descartes_model_.reset(new descartes_moveit::MoveitStateAdapter);
  auto* moveit_state_adapter = dynamic_cast<descartes_moveit::MoveitStateAdapter*>(descartes_model_.get());
  bool model_init = moveit_state_adapter->initialize(context_->planning_scene_monitor_->getRobotModel(), group_name,
                                                     world_frame, tcp_frame);

  if (!model_init)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Could not initialize robot model.");
    return false;
  }

  descartes_model_->setCheckCollisions(true);

  // If all went well, update our local copies of the parameters used to intiialize our descartes_model
  current_group_name_ = group_name;
  current_world_frame_ = world_frame;
  current_tcp_frame_ = tcp_frame;

  return true;
}

bool MoveGroupDescartesPathService::computeService(moveit_msgs::GetCartesianPath::Request& req,
                                                   moveit_msgs::GetCartesianPath::Response& res)
{
  ROS_INFO_NAMED(name_, "Received request to compute Descartes path");

  // Since Descartes takes in more parameters than are available in the moveit_msgs::GetCartesianPath::Request,
  // we provide rosparam interfaces that will read in additional prameters from the parameter server.
  nh_.param<double>("descartes_params/positional_tolerance", positional_tolerance_, 0.0);
  nh_.param<double>("descartes_params/positional_tolerance_inc", positional_tolerance_increment_, 0.0);
  nh_.param<double>("descartes_params/roll_orientation_tolerance", roll_orientation_tolerance_, 0.0);
  nh_.param<double>("descartes_params/pitch_orientation_tolerance", pitch_orientation_tolerance_, 0.0);
  nh_.param<double>("descartes_params/yaw_orientation_tolerance", yaw_orientation_tolerance_, 0.0);
  nh_.param<double>("descartes_params/orientation_tolerance_inc", orientation_tolerance_increment_, 0.0);

  // Get most up to date planning scene information
  context_->planning_scene_monitor_->updateFrameTransforms();

  const std::string& default_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();

  // TODO: check if this results in double transform.
  std::string world_frame = (req.header.frame_id.empty() ? default_frame : req.header.frame_id);
  if (current_group_name_ != req.group_name || current_world_frame_ != world_frame ||
      current_tcp_frame_ != req.link_name)
  {
    if (!initializeDescartesModel(req.group_name, world_frame, req.link_name))
      return false;
  }

  // Setup Descartes parameters
  descartes_planner::DensePlanner descartes_planner;
  descartes_planner.initialize(descartes_model_);

  const robot_model::JointModelGroup* jmg;
  std::vector<double> current_joints;
  {
    robot_state::RobotState start_state =
        planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
    robot_state::robotStateMsgToRobotState(req.start_state, start_state);

    jmg = start_state.getJointModelGroup(req.group_name);
    if (jmg == nullptr)
    {
      ROS_ERROR_NAMED(name_, "Invalid group name");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return true;
    }
    // Copy current joint positions from robot state to a current_joints vector for use outside of this scope
    start_state.copyJointGroupPositions(req.group_name, current_joints);
  }  // Planning scene lock released

  Eigen::Isometry3d current_pose;
  descartes_model_->getFK(current_joints, current_pose);

  if (req.waypoints.empty())
  {
    ROS_ERROR_NAMED(name_, "Must provide at least 1 input trajectory point %zu provided", req.waypoints.size());
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return true;
  }

  if (verbose_debug_)
    printJointsNamed("Current joints", current_joints);

  std::string link_name = req.link_name;
  if (link_name.empty() && !jmg->getLinkModelNames().empty())
    link_name = jmg->getLinkModelNames().back();

  if (verbose_debug_)
  {
    ROS_DEBUG_NAMED(name_, "Starting at current pose");

    std::string sep = "\n-----------------------------------------------------\n";
    ROS_DEBUG_STREAM_NAMED(name_, "current_pose\n"
                                      << "Position: \n"
                                      << current_pose.translation() << "\nRotation: \n"
                                      << current_pose.rotation() << sep);
  }

  bool no_transform =
      req.header.frame_id.empty() || robot_state::Transforms::sameFrame(req.header.frame_id, default_frame);

  EigenSTL::vector_Isometry3d waypoints(req.waypoints.size() + 1);
  waypoints[0] = current_pose;
  if (no_transform)
  {
    for (std::size_t i = 0; i < req.waypoints.size(); ++i)
      tf::poseMsgToEigen(req.waypoints[i], waypoints[i + 1]);
  }
  else
  {
    if (!transformWaypointsToFrame(req, default_frame, waypoints))
    {
      ROS_ERROR_NAMED(name_, "Error encountered transforming waypoints to frame '%s'", default_frame.c_str());
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
      return true;
    }
  }

  if (req.max_step < std::numeric_limits<double>::epsilon())
  {
    ROS_ERROR_NAMED(name_, "Maximum step to take between consecutive configurations along Descartes path was not "
                           "specified (this value needs to be > 0)");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return true;
  }

  bool global_frame = !robot_state::Transforms::sameFrame(link_name, req.header.frame_id);
  ROS_INFO_NAMED(name_,
                 "Attempting to follow %u waypoints for link '%s' using a step of %lf m and jump threshold %lf (in "
                 "%s reference frame)",
                 (unsigned int)waypoints.size(), link_name.c_str(), req.max_step, req.jump_threshold,
                 global_frame ? "global" : "link");

  // For each set of sequential waypoints we need to ensure that we do not exceed the req.max_step so we resample
  EigenSTL::vector_Isometry3d dense_waypoints;
  // Add the first point then add all the interpolated dense values
  dense_waypoints.push_back(waypoints[0]);
  for (std::size_t i = 1; i < waypoints.size(); ++i)
  {
    // If there are two identical waypoints in a row, then the trajectory generated will have identical points
    // planned for the same timestamp which could cause trajectory execution to fail.
    if (!waypoints[i - 1].isApprox(waypoints[i]))
      createDensePath(waypoints[i - 1], waypoints[i], req.max_step, dense_waypoints);
  }

  if (visual_debug_)
  {
    visual_tools_->deleteAllMarkers();
    visual_tools_->publishAxisPath(dense_waypoints, rviz_visual_tools::scales::XXSMALL);
    visual_tools_->publishPath(dense_waypoints, rviz_visual_tools::colors::TRANSLUCENT_DARK,
                               rviz_visual_tools::scales::SMALL);
    visual_tools_->trigger();
  }

  // Create a descartes trajectory with the current joint position at the first position of the
  // dense trajectory. This tells descartes to plan from this particular configuration rather
  // than just the starting end effector pose.
  std::vector<descartes_core::TrajectoryPtPtr> descartes_trajectory;
  descartes_core::TrajectoryPtPtr descartes_point =
      descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(current_joints));
  descartes_trajectory.push_back(descartes_point);
  createDescartesTrajectory(dense_waypoints, descartes_trajectory);

  // Use Descartes to solve path
  bool valid_path = true;
  std::vector<descartes_core::TrajectoryPtPtr> descartes_result;
  if (!descartes_planner.planPath(descartes_trajectory))
  {
    valid_path = false;
    ROS_INFO_STREAM_NAMED(name_, "Could not solve for a valid path.");
  }
  else
  {
    if (verbose_debug_)
      ROS_INFO_STREAM_NAMED(name_, "Found a valid path.");
  }

  if (!descartes_planner.getPath(descartes_result))
  {
    valid_path = false;
    ROS_INFO_STREAM_NAMED(name_, "Could not retrieve path.");
  }

  if (valid_path && verbose_debug_)
    ROS_INFO_STREAM_NAMED(name_, "Full path length = " << descartes_result.size());

  if (!valid_path)
  {
    ROS_INFO_STREAM_NAMED(name_, "Unable to generate a plan using Descartes.");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    res.fraction = 0.0;
    return true;
  }

  robot_trajectory::RobotTrajectory robot_trajectory(context_->planning_scene_monitor_->getRobotModel(),
                                                     req.group_name);

  res.fraction = copyDescartesResultToRobotTrajectory(descartes_result, req, robot_trajectory);

  // Time trajectory
  // TODO optionally compute timing to move the eef with constant speed
  trajectory_processing::IterativeParabolicTimeParameterization time_param;
  time_param.computeTimeStamps(robot_trajectory);
  if (verbose_debug_)
  {
    std::deque<double> durations = robot_trajectory.getWayPointDurations();
    double total_duration = 0;
    for (double duration : durations)
      total_duration += duration;
    ROS_DEBUG_NAMED(name_, "Total path duration: %.3f", total_duration);
  }

  robot_trajectory.getRobotTrajectoryMsg(res.solution);

  if (display_computed_paths_ && robot_trajectory.getWayPointCount() > 0)
    visual_tools_->publishTrajectoryPath(robot_trajectory, false);

  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;
}

void MoveGroupDescartesPathService::printDelta(const std::vector<double>& joints1, const std::vector<double>& joints2)
{
  if (joints1.size() != joints2.size())
  {
    ROS_ERROR_NAMED(name_, "printDelta received vectors of mismatched size");
    return;
  }
  std::stringstream o;
  for (std::size_t i = 0; i < joints1.size(); ++i)
  {
    o << "\t" << std::fixed << std::setw(5) << std::setprecision(3) << joints2[i] - joints1[i];
  }
  ROS_DEBUG_STREAM_NAMED(name_, o.str());
}

void MoveGroupDescartesPathService::printJoints(const std::vector<double>& joints)
{
  std::stringstream o;

  for (double joint : joints)
  {
    o << "\t" << std::fixed << std::setw(6) << std::setprecision(3) << joint;
  }
  ROS_DEBUG_STREAM_NAMED(name_, o.str());
}

void MoveGroupDescartesPathService::printJointsNamed(const std::string& name, const std::vector<double>& joints1)
{
  std::stringstream o;
  o << name;
  for (double i : joints1)
  {
    o << "\t" << std::fixed << std::setw(6) << std::setprecision(3) << i;
  }
  ROS_DEBUG_STREAM_NAMED(name_, o.str());
}
}  // end namespace descartes_capability

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(descartes_capability::MoveGroupDescartesPathService, move_group::MoveGroupCapability)
