FROM ros:jazzy-ros-core

# List of packages to install
RUN apt-get update && apt-get install -y \
  vim \
  git \
  ros-jazzy-nav2-bringup \
  ros-jazzy-navigation2 \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-apriltag-ros \
  ros-jazzy-image-proc \
  ros-jazzy-robot-localization \
  ros-jazzy-tf2-tools \
  python3-pip \
  sudo \
  ros-dev-tools

# Create workspace
RUN mkdir /ros_ws

RUN python3 -m pip install --upgrade pip

# Add a new user with the same UID and GID as the host user
ARG USER_ID
ARG GROUP_ID
RUN groupadd -g ${GROUP_ID} jos && \
  useradd -m -u ${USER_ID} -g jos jos
RUN usermod -aG sudo jos

# Change ownership of the workspace to the new user
RUN chown -R jos:jos /ros_ws
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/jos/.bashrc


# install dependencies from sphero-sdk
# WORKDIR /ros_ws/src/sphero_rvr/sphero-sdk-raspberrypi-python/
# RUN pip3 install --user -r /ros_ws/src/sphero_rvr/sphero-sdk-raspberrypi-python/requirements.txt
# RUN source ~/.profile

# Switch to the new user
USER jos
WORKDIR /ros_ws

ENTRYPOINT ["/bin/bash"]
