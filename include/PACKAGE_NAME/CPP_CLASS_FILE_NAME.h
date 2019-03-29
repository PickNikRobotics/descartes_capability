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

/* Author: FIRST_NAME LAST_NAME
   Desc: TODO(GITHUB_NAME):
*/

#ifndef PACKAGE_NAME__CPP_CLASS_FILE_NAME__H
#define PACKAGE_NAME__CPP_CLASS_FILE_NAME__H

#include <string>

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace PACKAGE_NAME
{
class CPP_CLASS_NAME
{
public:
  /** \brief Constructor */
  CPP_CLASS_NAME();

private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_ = "CPP_SHORT_NAME";

  // A shared node handle
  ros::NodeHandle nh_;
};  // end class CPP_CLASS_NAME

// Create std pointers for this class
typedef std::shared_ptr<CPP_CLASS_NAME> CPP_CLASS_NAMEPtr;
typedef std::shared_ptr<const CPP_CLASS_NAME> CPP_CLASS_NAMEConstPtr;

}  // namespace PACKAGE_NAME
#endif  // PACKAGE_NAME__CPP_CLASS_FILE_NAME__H
