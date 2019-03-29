#!/usr/bin/env python

"""
*********************************************************************
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
*********************************************************************

 Author: FIRST_NAME LAST_NAME
 Desc: TODO(GITHUB_NAME):

"""

import rospy

from std_msgs.msg import String

__logger_file_name = 'PYTHON_CLASS_FILE_NAME'

class PYTHON_CLASS_NAME(object):
    """TODO: Fill in docstring for PYTHON_CLASS_NAME."""

    def __init__(self):
        super(PYTHON_CLASS_NAME, self).__init__()
        self._node_name = 'PYTHON_CLASS_NAME_node'
        self._topic_name = '/PYTHON_CLASS_NAME_topic'
        rospy.init_node(self._node_name, anonymous=True)
        self._msg = String('hello world')
        self._publisher = rospy.Publisher(self._topic_name, String, queue_size=10)

    def run_node(self):
        rate = rospy.Rate(10)  # 10hz
        msg_count = 0
        while not rospy.is_shutdown():
            rospy.loginfo('{}: Publishing msg #: {}'.format(__logger_file_name, msg_count))
            self._publisher.publish(self._msg)
            msg_count += 1
            rate.sleep()

