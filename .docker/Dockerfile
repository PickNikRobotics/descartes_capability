# descartes_capability:melodic-source
# Downloads the source code, installs the remaining debian dependencies, and
# builds the package

FROM ros:melodic

# Remove warning 'Could not determine the width of the terminal.'
ENV TERM xterm

ENV CATKIN_WS=/root/ws_catkin
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Install SHH
# Note: --fix-missing is run to fix an issue with an outdated ros-melodic-rviz-visual-tools
RUN apt-get -qq update --fix-missing && \
    apt-get -qq install -y wget ssh python-catkin-tools python-wstool

# Setup for ssh onto github & bitbucket
# Before running Docker, run locally: `cp ~/.ssh/id_rsa_unsafe id_rsa`
RUN mkdir -p /root/.ssh
COPY id_rsa /root/.ssh/id_rsa
RUN chmod 700 /root/.ssh/id_rsa

RUN ssh-keyscan -t rsa github.com >> ~/.ssh/known_hosts
RUN ssh-keyscan -t rsa bitbucket.org >> ~/.ssh/known_hosts

# Download descartes_capability source
RUN git clone git@github.com:PickNikRobotics/descartes_capability.git && \
    wstool init . && \
    wstool merge ./descartes_capability/descartes_capability.rosinstall && \
    wstool update && \
    rm /root/.ssh/id_rsa

# Note that because we're building on top of melodic-ci, there should not be
# any deps installed unless something has changed in the source code since the
# other container was made (they are triggered together so should only be
# one-build out of sync)
RUN rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false

# Configures environment
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8

# Status rate is limited so that just enough info is shown to keep Docker from
# timing out, but not too much such that the Docker log gets too long (another
# form of timeout)
RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --jobs 1 --limit-status-rate 0.001 --no-notify && \
    echo '. /opt/ros/melodic/setup.bash' >> ~/.bashrc && \
    echo '. $CATKIN_WS/devel/setup.bash' >> ~/.bashrc
