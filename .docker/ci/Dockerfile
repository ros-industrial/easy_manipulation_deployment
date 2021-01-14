FROM ros:foxy-ros-base-focal

ENV MOVEIT_WS=/opt/moveit2
WORKDIR ${MOVEIT_WS}

# Install dependencies
RUN apt-get update -qq && \
    apt-get dist-upgrade -qq -y && \
    apt-get install -qq python3-vcstool \
                        curl && \
    mkdir src && \
    curl https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos --output moveit2.repos && \
    vcs import src < moveit2.repos && \
    # Use latest ros2_control and remove fake_joint
    touch src/fake_joint/COLCON_IGNORE && \
    git -C src/ros2_control reset master --hard && \
    git -C src/ros2_controllers reset master --hard && \
    # Remove folders declared as COLCON_IGNORE
    find -L . -name COLCON_IGNORE -printf "%h\0" | xargs -0 rm -rf && \
    rosdep update --rosdistro "$ROS_DISTRO" && \
    rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -yr && \
    rm -rf /var/lib/apt/lists/* \
           src/*

# Install Moveit2 from source
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh && \
    vcs import src < moveit2.repos && \
    # Use latest ros2_control and remove fake_joint
    touch src/fake_joint/COLCON_IGNORE && \
    git -C src/ros2_control reset master --hard && \
    git -C src/ros2_controllers reset master --hard && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=RELEASE && \
      rm -rf src build log

# Set Work Directory
WORKDIR /
