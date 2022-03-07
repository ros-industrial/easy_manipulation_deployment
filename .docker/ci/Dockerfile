FROM ros:rolling

ENV MOVEIT_WS=/opt/moveit2
WORKDIR ${MOVEIT_WS}

# Install dependencies
RUN apt-get update -qq && \
    apt-get upgrade -qq -y && \
    apt-get install -qq python3-vcstool \
                        curl && \
    mkdir src && \
    curl https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos --output moveit2.repos && \
    vcs import src < moveit2.repos && \
    git clone https://github.com/ros-planning/moveit2 src/moveit2 --depth 1 && \ 
    # Remove folders declared as COLCON_IGNORE
    find -L . -name COLCON_IGNORE -printf "%h\0" | xargs -0 rm -rf && \
    rosdep update --rosdistro "$ROS_DISTRO" && \
    rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -yr && \
    rm -rf /var/lib/apt/lists/* \
           src/*

# Install Moveit2 from source
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh && \
    vcs import src < moveit2.repos && \
    git clone https://github.com/ros-planning/moveit2 src/moveit2 --depth 1 && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=RELEASE && \
      rm -rf src build log

# Setup entrypoint
WORKDIR /
COPY ./moveit2_entrypoint.sh /

ENTRYPOINT ["/moveit2_entrypoint.sh"]
CMD ["bash"]
