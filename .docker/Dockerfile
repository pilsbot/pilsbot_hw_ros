FROM dustynv/ros:foxy-ros-base-l4t-r32.4.4

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
  git clone -b ros2 https://github.com/ros-drivers/ackermann_msgs.git && \
  git clone -b ros2 https://github.com/ros/xacro.git && \
  git clone -b ${ROS_DISTRO} https://github.com/ros-controls/ros2_control.git && \
  git clone -b ${ROS_DISTRO} https://github.com/ros2/teleop_twist_joy && \
  git clone -b ${ROS_DISTRO} https://github.com/ros/diagnostics && \
  git clone -b ros2 https://github.com/ros-drivers/joystick_drivers && \
  # remove for now
  # git clone -b ${ROS_DISTRO} https://github.com/ros-controls/ros2_controllers.git && \
  git clone -b ${ROS_DISTRO}-devel https://github.com/ros-controls/control_msgs.git && \
  git clone -b ${ROS_DISTRO}-devel https://github.com/ros-controls/realtime_tools.git && \
  cd .. \
  && rosdep install --from-paths src --ignore-src --skip-keys libsdl2-dev -r -y
# This is needed, as joystick_drivers can't use libsdl2 in Ubuntu 18.04 but fails trying.
# However, it will happily work when it's not there.
RUN apt remove libsdl2-dev -y

RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh

RUN source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --symlink-install

RUN cd src && \
  git clone --recursive -b ${ROS_DISTRO}-devel https://github.com/pilsbot/pilsbot_hw_ros.git && \
  git clone https://github.com/pilsbot/pilsbot_description.git && \
  git clone --recursive https://github.com/pilsbot/pils_control.git && \
  cd .. \
  && rosdep install --from-paths src --ignore-src --skip-keys="libsdl2-dev" -r -y

RUN apt remove libsdl2-dev -y

RUN source /opt/ros/${ROS_DISTRO}/install/setup.bash && source install/setup.bash && \
    colcon build --symlink-install 

RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh
RUN touch /root/.bashrc && \ 
  echo "source /colcon_ws/install/setup.bash \n" >> /root/.bashrc

RUN rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /

RUN sed -i \
    's/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/source "\/opt\/ros\/$ROS_DISTRO\/install\/setup.bash"/g' \
    /ros_entrypoint.sh && \
    cat /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
