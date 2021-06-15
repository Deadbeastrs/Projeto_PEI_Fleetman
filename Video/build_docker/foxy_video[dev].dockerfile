FROM ros:foxy
ENV ROS_DOMAIN_ID=5

# Install required/useful packages
RUN apt update && apt upgrade -y && apt install \
    libasio-dev \
    libjsoncpp-dev \
    python3-pip \
    tmux \
    vim -y

RUN apt-get install -qqy x11-apps

RUN /bin/bash -c "sudo -H pip3 install --upgrade pip"

#RUN pip3 install flask opencv-python

# Create workspace dir
ARG WS_DIR=/ws
#RUN mkdir -p $WS_DIR/h264_utils && mkdir -p $WS_DIR/libvideogstreamer && mkdir -p $WS_DIR/libvideolibav && mkdir -p $WS_DIR/x264_image_transport
RUN mkdir -p $WS_DIR/teste
RUN /bin/bash -c "cd $WS_DIR && sudo git clone https://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@github.com/Deadbeastrs/Projeto_PEI_Fleetman_private.git"

# Record build time version
#ARG TMP_BUILD_ARG=undefined
#ENV BUILD_TIMESTAMP=$TMP_BUILD_ARG

# Copy source files, scripts and configs
#COPY h264_utils $WS_DIR/h264_utils
#COPY libvideogstreamer $WS_DIR/libvideogstreamer
#COPY libvideolibav /$WS_DIR/libvideolibav
#COPY x264_image_transport /$WS_DIR/x264_image_transport
#COPY ros-video-streaming /$WS_DIR/ros-video-streaming

# Build drone code and update bashrc
#RUN /bin/bash -c "cd  $WS_DIR && source /opt/ros/$ROS_DISTRO/setup.bash && \
#    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
#    echo "source  $WS_DIR/install/setup.bash" >> /root/.bashrc"

RUN /bin/bash -c "sudo apt-get install ros-foxy-sensor-msgs ros-foxy-cv-bridge ros-foxy-image-transport -y"

WORKDIR $WS_DIR
#CMD ["/bin/bash", "-c", "source install/setup.bash && ./run_drone_controller.sh $DRONE_ID $CFG_FILE"]   
