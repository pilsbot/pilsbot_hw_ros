FROM ros:foxy-ros-base-l4t-r32.4.4

ENV ROS_DISTRO foxy

RUN apt -q -qq update && \
  DEBIAN_FRONTEND=noninteractive apt install -y \
  software-properties-common \
  wget

RUN apt -q -qq update && \
  DEBIAN_FRONTEND=noninteractive apt install -y --allow-unauthenticated \
  python3-rosinstall

RUN rosdep update

RUN mkdir -p /colcon_ws/src

WORKDIR /colcon_ws

RUN cd src && \
  # git clone --recursive -b ${ROS_DISTRO}-devel https://github.com/pilsbot/pilsbot_hw_ros.git && \
  git clone https://github.com/pilsbot/pilsbot_description.git && \
  git clone -b dashing-devel https://github.com/ros/xacro.git && \
  git clone https://github.com/ros-controls/ros2_control.git && \
  git clone https://github.com/ros-controls/ros2_controllers.git && \
  git clone -b ${ROS_DISTRO}-devel https://github.com/ros-controls/control_msgs.git && \
  git clone -b ${ROS_DISTRO}-devel https://github.com/ros-controls/realtime_tools.git && \
  cd .. 
  # && rosdep install --from-paths src --ignore-src -r -y

RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh

RUN source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --symlink-install --packages-up-to pilsbot_hw_ros

RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh
RUN touch /root/.bashrc && \ 
  echo "source /colcon_ws/install/local_setup.bash\n" >> /root/.bashrc

RUN rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /

RUN sed -i \
    's/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/source "\/opt\/ros\/$ROS_DISTRO\/install\/setup.bash"/g' \
    /ros_entrypoint.sh && \
    cat /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
