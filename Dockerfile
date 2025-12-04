FROM ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8

WORKDIR /ros2_ws

RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    build-essential \
    locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    ros-humble-geographic-msgs \
    python3-colcon-common-extensions

RUN apt-get install -y \
    git

RUN mkdir src && cd src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    cd px4_msgs && \
    git checkout 1ebaae4

RUN apt-get update && \
    apt-get install -y curl lsb-release gnupg && \
    curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null 

RUN apt-get update && \
    apt-get install -y \
    gz-harmonic

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

COPY src src

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install geographiclib
RUN pip3 install transforms3d
RUN pip3 install opencv-contrib-python==4.11.0.86


COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV GZ_DISCOVERY_SERVER=127.0.0.1:11345
ENV GZ_PARTITION=px4
ENV GZ_RELAY=1

ENTRYPOINT ["/entrypoint.sh"]