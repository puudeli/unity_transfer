FROM ros:noetic-ros-base

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Create a Catkin workspace
RUN apt-get update \
 && apt-get install -y wget unzip python-is-python3 ros-noetic-tf2-ros ros-noetic-control-msgs \
 && rm -rf /var/lib/apt/lists/* \
 && source /opt/ros/noetic/setup.bash \
 && mkdir -p /unity_transfer_ws/src \
 && cd /unity_transfer_ws/src \
 && catkin_init_workspace \
# get unity_transfer package
 && wget https://github.com/puudeli/unity_transfer/archive/refs/heads/main.zip \
 && unzip main.zip \
 && mv unity_transfer-main unity_transfer \
 && rm main.zip \
# get ROS-TCP-Endpoint package
 && wget https://github.com/Unity-Technologies/ROS-TCP-Endpoint/archive/refs/tags/v0.7.0.zip \
 && unzip v0.7.0.zip \
 && mv ROS-TCP-Endpoint-0.7.0 ROS-TCP-Endpoint \
 && rm v0.7.0.zip \
# Build the Catkin workspace and ensure it's sourced
 && source /opt/ros/noetic/setup.bash \
 && cd /unity_transfer_ws \
 && catkin_make

RUN echo "source /unity_transfer_ws/devel/setup.bash" >> ~/.bashrc

# Set the working folder at startup
WORKDIR /unity_transfer_ws