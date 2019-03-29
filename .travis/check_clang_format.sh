# Install Dependencies
travis_run apt-get -qq install -y clang-format-3.9

# Change to source directory.
travis_run cd $CI_SOURCE_PATH
travis_run ls -la

# This directory can have its own .clang-format config file but if not, MoveIt's will be provided
if [ ! -f .clang-format ]; then
    wget "https://raw.githubusercontent.com/ros-planning/moveit/$ROS_DISTRO-devel/.clang-format"
fi

# Run clang-format
echo "Running clang-format"
find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format-3.9 -i -style=file

echo "Showing changes in code style:"
git --no-pager diff

# Make sure no changes have occured in repo
if ! git diff-index --quiet HEAD --; then
    # changes
    echo "clang-format test failed: changes required to comply to formatting rules. See diff above.";
    exit 1 # error
fi

echo "Passed clang-format test"
