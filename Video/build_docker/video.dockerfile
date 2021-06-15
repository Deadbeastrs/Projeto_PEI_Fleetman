FROM ros:dashing
ENV ROS_DOMAIN_ID=5

# Create workspace dir
ARG WS_DIR=/ws
COPY repos.key $WS_DIR/repos.key
COPY scripts $WS_DIR/scripts
COPY server $WS_DIR/server
COPY x264_software_ws $WS_DIR/x264_software_ws
COPY x264_ws $WS_DIR/x264_ws

RUN /bin/bash -c "cd /ws && sudo apt-key add repos.key"

# Install required/useful packages
RUN apt update && apt upgrade -y && apt install \
    libasio-dev \
    libjsoncpp-dev \
    python3-pip \
    tmux \
    vim -y

RUN apt-get update 
    
RUN apt-get -y install xauth

RUN apt-get install -qqy x11-apps

RUN /bin/bash -c "sudo -H pip3 install --upgrade pip"

RUN pip3 install flask opencv-python Flask Flask-SocketIO Flask-Cors

RUN apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio libgstrtspserver-1.0-dev gstreamer1.0-rtsp pulseaudio gstreamer1.0-plugins-base-apps -y

#RUN mkdir -p $WS_DIR/h264_utils && mkdir -p $WS_DIR/libvideogstreamer && mkdir -p $WS_DIR/libvideolibav && mkdir -p $WS_DIR/x264_image_transport
RUN mkdir -p $WS_DIR/teste

# Build drone code and update bashrc
RUN /bin/bash -c 'cd  $WS_DIR && source /opt/ros/$ROS_DISTRO/setup.bash && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc'

RUN /bin/bash -c "sudo ln -s /usr/include/jsoncpp/json/ /usr/include/json"

RUN /bin/bash -c "sudo apt-get install ros-dashing-sensor-msgs ros-dashing-cv-bridge ros-dashing-image-transport -y"

WORKDIR $WS_DIR
#CMD ["/bin/bash", "-c", "cd $WS_DIR/Projeto_PEI_Fleetman_private/x264_ws/src/x264/ && colcon build --cmake-args '-DHWACC=0' && source install/setup.bash"]
EXPOSE 8887
