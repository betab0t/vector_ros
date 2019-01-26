FROM ros:melodic

ARG anki_user_email
ARG anki_user_password
ARG vector_ip
ARG vector_name
ARG vector_serial

# Init ROS and setup a workspace
RUN bash ./ros_entrypoint.sh
RUN mkdir -p /catkin_ws/src/vector_ros

# Install required dependencies for ROS-Python3.6 nodes
RUN apt-get update && apt-get install -y \
    python3-yaml \
    python3-pip \
    expect

RUN pip3 install \
    rospkg \
    catkin_pkg

# Prepare Excpet script used to configure the SDK
RUN printf '#!/usr/bin/expect -f\n\
    \rspawn python3.6 -m anki_vector.configure -e $env(env_anki_user_email) -i $env(env_vector_name) -n $env(env_vector_ip) -s $env(env_vector_serial)\n\
    \rexpect "Do you wish to proceed? \\(y/n\\) "\n\
    \rsend "y\\n"\n\
    \rexpect "Enter Password: "\n\
    \rsend "$env(env_anki_user_password)\\r"\n\
    \rexpect "SUCCESS!"' >> configure.sh && chmod +x configure.sh

# Pass args to script as environment variables
ENV env_anki_user_email=$anki_user_email
ENV env_anki_user_password=$anki_user_password
ENV env_vector_ip=$vector_ip
ENV env_vector_name=$vector_name
ENV env_vector_serial=$vector_serial

# Setup Anki's SDK
RUN python3 -m pip install --user anki_vector && ./configure.sh && rm configure.sh

WORKDIR /catkin_ws

CMD bash