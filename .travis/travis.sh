#!/bin/bash -eu

# Software License Agreement - BSD License
#
# This a fork of moveit_ci that supports private repos.
#
# Inspired by JSK travis https://github.com/jsk-ros-pkg/jsk_travis
# Inspired by ROS Industrial https://github.com/ros-industrial/industrial_ci
#
# Author:  Dave Coleman, Isaac I. Y. Saito, Robert Haschke

export CI_SOURCE_PATH=$(pwd) # The repository code in this pull request that we are testing
export CI_PARENT_DIR=.travis  # This is the folder name that is used in downstream repositories in order to point to this repo.
export HIT_ENDOFSCRIPT=false
export REPOSITORY_NAME=${PWD##*/}
cd ..
export CATKIN_WS=${PWD}/ws_moveit
echo "---"
echo "Testing branch '$TRAVIS_BRANCH' of '$REPOSITORY_NAME' on ROS '$ROS_DISTRO'"

# Helper functions
source ${CI_SOURCE_PATH}/$CI_PARENT_DIR/util.sh

# Assign value if variable is unset or null to prevent strict mode error
: "${IN_DOCKER:=0}"

# Run all CI in a Docker container
if [ "$IN_DOCKER" -eq 0 ]; then

  # Install wstool. Pipe to null to avoid a lot of noise and problems with pip
  sudo -H pip install --upgrade pip  > /dev/null
  sudo pip install -U wstool  > /dev/null

  # Create workspace in root Travis environment where the SSH keys are easily accessible
  travis_run mkdir -p $CATKIN_WS/src
  travis_run cd $CATKIN_WS/src

  # Move repo we are testing into catkin ws
  sudo mv $CI_SOURCE_PATH $CATKIN_WS/src
  export CI_SOURCE_PATH="$CATKIN_WS/src/$REPOSITORY_NAME"

  # Install dependencies necessary to run build using .rosinstall files
  if [ ! "$UPSTREAM_WORKSPACE" ]; then
    export UPSTREAM_WORKSPACE="debian";
  fi
  case "$UPSTREAM_WORKSPACE" in
    debian)
      echo "Obtain deb binary for upstream packages."
      ;;
    http://* | https://*) # When UPSTREAM_WORKSPACE is an http url, use it directly
      travis_run wstool init .
      # Handle multiple rosintall entries.
      IFS=','  # Multiple URLs can be given separated by comma.
      for rosinstall in $UPSTREAM_WORKSPACE; do
        travis_run wstool merge -k $rosinstall
      done
      ;;
    *) # Otherwise assume UPSTREAM_WORKSPACE is a local file path
      travis_run wstool init .

      if [ -e $CI_SOURCE_PATH/$UPSTREAM_WORKSPACE ]; then
        # install (maybe unreleased version) dependencies from source
        travis_run wstool merge file://$CI_SOURCE_PATH/$UPSTREAM_WORKSPACE
      else
        echo "No rosinstall file found, aborting" && exit 1
      fi
      ;;
  esac

  # Download upstream packages into workspace
  if [ -e .rosinstall ]; then
    # ensure that the downstream is not in .rosinstall
    # the exclamation mark means to ignore errors
    travis_run_true wstool rm $REPOSITORY_NAME
    cat .rosinstall
    travis_run wstool update
  fi

  # Debug: see the files in current folder
  travis_run ls -a

  # Choose the correct CI container to use
  case "$ROS_REPO" in
    ros-shadow-fixed)
      export DOCKER_IMAGE=moveit/moveit:$ROS_DISTRO-ci-shadow-fixed
      ;;
    *)
      export DOCKER_IMAGE=moveit/moveit:$ROS_DISTRO-ci
      ;;
  esac
  echo "Starting Docker image: $DOCKER_IMAGE"

  # Pull first to allow us to hide console output
  docker pull $DOCKER_IMAGE > /dev/null

  # Start Docker container
  docker run \
         -e ROS_REPO \
         -e ROS_DISTRO \
         -e BEFORE_SCRIPT \
         -e CI_PARENT_DIR \
         -e CI_SOURCE_PATH \
         -e TRAVIS_BRANCH \
         -e TEST \
         -e TEST_BLACKLIST \
         -v $HOME/.ccache:/root/.ccache \
         -v $CATKIN_WS:/root/ws_moveit \
         $DOCKER_IMAGE \
         /bin/bash -c "cd /root/ws_moveit/src/$REPOSITORY_NAME; source $CI_PARENT_DIR/travis.sh;"
  return_value=$?

  if [ $return_value -eq 0 ]; then
    echo "$DOCKER_IMAGE container finished successfully"
    HIT_ENDOFSCRIPT=true;
    exit 0
  fi
  echo "$DOCKER_IMAGE container finished with errors"
  exit 1 # error
fi

# If we are here, we can assume we are inside a Docker container
echo "Inside Docker container"

# Update the sources
travis_run apt-get -qq update
travis_run apt-get -qq upgrade

# Make sure the packages are up-to-date
#travis_run apt-get -qq dist-upgrade

# Split for different tests
for t in $TEST; do
    case "$t" in
        "clang-format")
            source ${CI_SOURCE_PATH}/$CI_PARENT_DIR/check_clang_format.sh || exit 1
            exit 0 # This runs as an independent job, do not run regular travis test
        ;;
    esac
done

# Enable ccache for faster builds
travis_run apt-get -qq install ccache
export PATH=/usr/lib/ccache:$PATH

# Install and run xvfb to allow for X11-based unittests on DISPLAY :99
# travis_run apt-get -qq install xvfb mesa-utils
# Xvfb -screen 0 640x480x24 :99 &
# export DISPLAY=:99.0
# travis_run_true glxinfo

# Setup rosdep - note: "rosdep init" is already setup in base ROS Docker image
travis_run rosdep update

# TODO: restore run before script
set -x
pwd
ls -a
set +x

# Install source-based package dependencies
travis_run rosdep install -y -q -n --from-paths . --ignore-src --rosdistro $ROS_DISTRO

# Change to base of workspace
travis_run cd ..

# Configure catkin
travis_run catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O3"

# Blacklist unnecessary packages
# travis_run catkin config --blacklist moveit_setup_assistant moveit_fake_controller_manager moveit_planners_chomp  moveit_ros_benchmarks moveit_commander moveit_controller_manager_example chomp_motion_planner

# Console output fix for: "WARNING: Could not encode unicode characters"
export PYTHONIOENCODING=UTF-8

# For a command that doesn’t produce output for more than 10 minutes, prefix it with travis_run_wait
travis_run_wait 60 catkin build --no-status --summarize || exit 1

travis_run ccache -s

# Source the new built workspace
travis_run source devel/setup.bash

# Choose which packages to run tests on
# echo "Test blacklist: $TEST_BLACKLIST"
# echo "--------------"
# TEST_PKGS=$(catkin_topological_order "$CI_SOURCE_PATH" --only-names | grep -Fvxf <(echo "$TEST_BLACKLIST" | tr ' ;,' '\n') | tr '\n' ' ')

# if [ -n "$TEST_PKGS" ]; then
#     TEST_PKGS="--no-deps $TEST_PKGS";
#     # Fix formatting of list of packages to work correctly with Travis
#     IFS=' ' read -r -a TEST_PKGS <<< "$TEST_PKGS"
# fi
# echo -e "Test packages: ${TEST_PKGS}"

# # Run catkin package tests
# travis_run catkin build --no-status --summarize --make-args tests -- ${TEST_PKGS[@]}

# # Run non-catkin package tests
# travis_run catkin build --catkin-make-args run_tests -- --no-status --summarize ${TEST_PKGS[@]}

# # Show failed tests
# for file in $(catkin_test_results | grep "\.xml:" | cut -d ":" -f1); do
#     travis_run cat $file
# done

# Run catkin package tests
travis_run catkin run_tests --no-deps descartes_capability

travis_run rosrun moveit_ros_move_group list_move_group_capabilities
travis_run cat build/descartes_capability/test_results/descartes_capability/rosunit-descartes_path_service_capability_test.xml

# Show test results summary and throw error if necessary
travis_run catkin_test_results build/descartes_capability

echo "Travis script has finished successfully"
HIT_ENDOFSCRIPT=true
exit 0
