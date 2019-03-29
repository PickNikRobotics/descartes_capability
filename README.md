# descartes_capability

Description: A move_group capability that uses Descartes for Cartesian Planning

<img src="https://picknik.ai/images/logo.jpg" width="100">

Developed by Mike Lautman at [PickNik Consulting](http://picknik.ai/)

TODO(mlautman): fix Travis badge:
[![Build Status](https://travis-ci.com/PickNikRobotics/descartes_capability.svg?token=o9hPQnr2kShM9ckDs6J8&branch=master)](https://travis-ci.com/PickNikRobotics/descartes_capability)

## Install

### Ubuntu Debian

> Note: this package has not been released yet

    sudo apt-get install ros-melodic-descartes_capability

### Build from Source

These instructions assume you are running on Ubuntu 16.04:

1. [Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) and the following build tools.

        sudo apt-get install python-wstool python-catkin-tools

1. Re-use or create a catkin workspace:

        export CATKIN_WS=~/ws_catkin/
        mkdir -p $CATKIN_WS
        cd $CATKIN_WS

1. Download the required repositories and install any dependencies:

        git clone git@github.com:PickNikRobotics/descartes_capability.git
        wstool init src
        wstool merge -t src descartes_capability/descartes_capability.rosinstall
        wstool update -t src
        rosdep install --from-paths src --ignore-src --rosdistro melodic

1. Configure and build the workspace:

        catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build

1. Source the workspace.

        source devel/setup.bash

## Developers: Quick update code repositories

To make sure you have the latest repos:

    cd $CATKIN_WS/src/descartes_capability
    git checkout master
    git pull origin master
    cd ..
    wstool merge descartes_capability/descartes_capability.rosinstall
    wstool update
    rosdep install --from-paths . --ignore-src --rosdistro melodic

## Setup

To use the Descartes path service capability in place of the move

## Run

Run descartes_path_service_capability_demo
```
roslaunch descartes_capability descartes_path_service_capability_demo.launch
```

### Run with Debuging

Run descartes_path_service_capability_demo with GDB
```
roslaunch descartes_capability descartes_path_service_capability_demo.launch debug:=true
```

Run descartes_path_service_capability_demo with Callgrind
```
roslaunch descartes_capability descartes_path_service_capability_demo.launch callgrind:=true
```

Run descartes_path_service_capability_demo with Valgrind
```
roslaunch descartes_capability descartes_path_service_capability_demo.launch callgrind:=true
```

## Run Docker

### Prerequisite

You must have a private rsa key `~/.ssh/id_rsa` that is not password protected and is attached to your Github and Bitbucket accounts. You must also have a working installation of `docker`.

1. Navigate to `$CATKIN_WS/src/descartes_capability/.docker`. You should see the `Dockerfile` recipe in the directory.

1. Build the docker image

        cd $CATKIN_WS/src/descartes_capability/.docker
        cp ~/.ssh/id_rsa id_rsa && docker build -t descartes_capability:melodic-source .; rm id_rsa

1. Run the docker image

    * Without the gui

            docker run -it --rm descartes_capability:melodic-source /bin/bash

    * With the gui (tested with Ubuntu native and a Ubuntu VM)

            . ./gui-docker -it --rm descartes_capability:melodic-source /bin/bash

## Code API

> Note: this package has not been released yet

See [the Doxygen documentation](http://docs.ros.org/melodic/api/descartes_capability/html/anotated.html)

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin lint -W2 --rosdistro melodic

Use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/) to run tests.

    catkin run_tests --no-deps --this -i
