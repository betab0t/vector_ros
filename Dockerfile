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
    expect \
    python-catkin-tools \
    python3-dev \
    python3-catkin-pkg-modules \
    python3-numpy \
    ros-melodic-cv-bridge

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
    \rspawn python3.6 -m anki_vector.configure -e $env(env_anki_user_email) -n $env(env_vector_name) -i $env(env_vector_ip) -s $env(env_vector_serial)\n\
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

# Build cv_bridge for Python3.6
RUN /ros_entrypoint.sh /bin/bash -c "mkdir /cv_bridge_build_ws && \
                                     cd /cv_bridge_build_ws && \
                                     catkin init && \
                                     catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3.6 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so && \
                                     catkin config --install && \
                                     git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv && \
                                     cd src/vision_opencv/ && \
                                     git checkout melodic && \
                                     cd ../../ && \
                                     catkin build cv_bridge"

WORKDIR /catkin_ws

CMD /bin/bash -c "catkin_make --pkg vector_ros && \
                  source /catkin_ws/devel/setup.bash && \
                  source /cv_bridge_build_ws/install/setup.bash --extend && \
                  export QT_X11_NO_MITSHM=1 && \
                  roslaunch vector_ros vector.launch"