FROM osrf/ros:melodic-desktop-full

ARG anki_user_email
ARG anki_user_password
ARG vector_ip
ARG vector_name
ARG vector_serial

# Init Catkin workspace
RUN mkdir -p /catkin_ws/src/vector_ros
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install required dependencies
RUN apt-get update && apt-get install -y \
    python3-yaml \
    python3-pip \
    expect

RUN pip3 install \
    rospkg \
    catkin_pkg

# Install up-to-date rosunit so we'll get the patch for Python3
RUN cd /catkin_ws/src/ && \
    git clone https://github.com/ros/ros && \
    cd .. && \
    /ros_entrypoint.sh catkin_make install --pkg rosunit

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

# Install and build diff_drive package
RUN cd /catkin_ws/src && \
    git clone https://github.com/merose/diff_drive && \
    cd .. && \
    /ros_entrypoint.sh catkin_make --pkg diff_drive

WORKDIR /catkin_ws

CMD /bin/bash -c "catkin_make --pkg vector_ros && source /catkin_ws/devel/setup.bash && roslaunch vector_ros vector.launch"