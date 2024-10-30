FROM ros:humble-ros-base

# List of packages to install
RUN apt-get update && apt-get install -y \
  vim \
  git \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-apriltag-ros \
  ros-humble-image-proc \
  ros-humble-robot-localization \
  ros-humble-tf2-tools \
  ros-humble-rviz2 \
  ros-humble-rqt \
  ros-humble-rqt-common-plugins \
  ros-humble-usb-cam \
  python3-pip \
  sudo \
  ros-dev-tools

# Create workspace
RUN mkdir /ros_ws

# RUN python3 -m pip install --upgrade pip
python3 -m pip install aiohttp requests websocket-client pytest-asyncio pytest twine pyserial pyserial-asyncio
 
# Add a new user with the same UID and GID as the host user
ARG USER_ID
ARG GROUP_ID
ARG USER_NAME
RUN groupadd -g ${GROUP_ID} ${USER_NAME} && \
  useradd -m -u ${USER_ID} -g ${USER_NAME} ${USER_NAME}
RUN usermod -aG sudo ${USER_NAME}

# Change ownership of the workspace to the new user
RUN chown -R ${USER_NAME}:${USER_NAME} /ros_ws
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USER_NAME}/.bashrc

# Switch to the new user
USER ${USER_NAME}
WORKDIR /ros_ws

ENTRYPOINT ["/bin/bash"]
