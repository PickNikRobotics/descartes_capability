/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Mike Lautman
   Desc: TODO(mlautman):
*/

/** EXAMPLES:
    EXPECT_FALSE(robot_state.hasFixedLinks());
    EXPECT_EQ(robot_state.getFixedLinksCount(), 0);
    EXPECT_TRUE(robot_state.getPrimaryFixedLink() == NULL);
    EXPECT_GT(robot_state.getFixedLinksMode(), 0);
    EXPECT_LT( fabs(vars[0] - 0), EPSILON) << "Virtual joint in wrong position " << vars[0];
*/

// C++
#include <string>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// For listing capabilities
#include <moveit/move_group/move_group_capability.h>
#include <pluginlib/class_loader.hpp>

// For loading panda robot description
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/utils/robot_model_test_utils.h>

namespace descartes_capability
{
class MoveGroupDescartesPathServiceTest : public ::testing::Test
{
public:
  MoveGroupDescartesPathServiceTest() : nh_("~")
  {
  }

protected:
  ros::NodeHandle nh_;
};  // class MoveGroupDescartesPathServiceTest

TEST_F(MoveGroupDescartesPathServiceTest, TestDescartesCapabilityAvailable)
{
  try
  {
    pluginlib::ClassLoader<move_group::MoveGroupCapability> capability_plugin_loader("moveit_ros_move_group",
                                                                                     "move_group::MoveGroupCapability");
    // Test a move_group default plugin
    EXPECT_TRUE(capability_plugin_loader.isClassAvailable("move_group/MoveGroupCartesianPathService"));
    // Test the Descartes capability
    EXPECT_TRUE(capability_plugin_loader.isClassAvailable("descartes_capability/MoveGroupDescartesPathService"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    std::cerr << "Exception while creating plugin loader for move_group capabilities: " << ex.what() << std::endl;
    EXPECT_TRUE(false);
  }
}

TEST_F(MoveGroupDescartesPathServiceTest, TestDescartesCapabilityCreation)
{
  try
  {
    pluginlib::ClassLoader<move_group::MoveGroupCapability> capability_plugin_loader("moveit_ros_move_group",
                                                                                     "move_group::MoveGroupCapability");

    move_group::MoveGroupCapabilityPtr cap =
        capability_plugin_loader.createUniqueInstance("descartes_capability/MoveGroupDescartesPathService");

    EXPECT_EQ("DescartesPathService", cap->getName());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    std::cerr << "Exception while creating plugin loader for move_group capabilities: " << ex.what() << std::endl;
    EXPECT_TRUE(false);
  }
}

TEST_F(MoveGroupDescartesPathServiceTest, TestDescartesCapabilityInitialize)
{
  try
  {
    pluginlib::ClassLoader<move_group::MoveGroupCapability> capability_plugin_loader("moveit_ros_move_group",
                                                                                     "move_group::MoveGroupCapability");

    bool allow_trajectory_execution = false;
    bool debug = false;
    planning_scene_monitor::PlanningSceneMonitorPtr psm(
        new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    move_group::MoveGroupContextPtr context(new move_group::MoveGroupContext(psm, allow_trajectory_execution, debug));

    move_group::MoveGroupCapabilityPtr cap =
        capability_plugin_loader.createUniqueInstance("descartes_capability/MoveGroupDescartesPathService");

    EXPECT_TRUE(capability_plugin_loader.isClassLoaded("descartes_capability/MoveGroupDescartesPathService"));
    cap->setContext(context);
    cap->initialize();
    EXPECT_EQ("DescartesPathService", cap->getName());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    std::cerr << "Exception while creating plugin loader for move_group capabilities: " << ex.what() << std::endl;
    EXPECT_TRUE(false);
  }
}

}  // namespace descartes_capability

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "descartes_path_service_capability");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();

  spinner.stop();
  ros::shutdown();
  return result;
}
