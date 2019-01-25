FROM ros:melodic

# Init ROS and setup a workspace
RUN bash ./ros_entrypoint.sh
RUN mkdir -p /catkin_ws/src/vector_ros
WORKDIR /catkin_ws

# Install required dependencies for ROS-Python3.6 nodes
RUN apt-get update && apt-get install -y \
    python3-yaml \
    python3-pip

RUN pip3 install \
    rospkg \
    catkin_pkg

CMD bash