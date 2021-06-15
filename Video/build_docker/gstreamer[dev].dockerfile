 # gstreamer
FROM ubuntu:bionic AS gst_build

RUN cp /etc/apt/sources.list /etc/apt/sources.list~ && sed -Ei 's/^# deb-src /deb-src /' /etc/apt/sources.list
RUN apt-get update

ENV TZ=Australia/Melbourne
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get build-dep -y gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-nice
RUN apt-get install -y ninja-build curl zip software-properties-common python3-pip flex git bison
RUN pip3 install meson

RUN curl -s https://gitlab.freedesktop.org/gstreamer/gst-build/-/archive/1.16/gst-build-1.16.zip > gst-build.zip
RUN unzip gst-build.zip
WORKDIR /gst-build-1.16

RUN meson subprojects download gst-plugins-bad

# hack to fix openexr plugin on 18.04
RUN sed -i 's/-std=c++98/-std=c++11/g' subprojects/gst-plugins-bad/ext/openexr/meson.build

RUN meson build/
RUN meson configure -Dgtk_doc=disabled -Dorc=disabled -Dexamples=disabled -Dnls=disabled build/
RUN ninja -C build/

FROM ubuntu:bionic

ENV TZ=Australia/Melbourne
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
ENV CHOOSE_ROS_DISTRO=dashing
RUN apt-get update && apt-get install -y ros-$CHOOSE_ROS_DISTRO-ros-base

RUN apt-get install -y python3-pip aptitude gettext
RUN aptitude install -y '?reverse-depends(gstreamer-1.0)'
RUN pip3 install meson

COPY --from=gst_build /gst-build-1.16/ /gst-build-1.16/
RUN apt-get install -y $(apt-cache depends gstreamer1.0-plugins-base gstreamer1.0-plugins-bad gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly | grep Depends | sed -e "s/.*ends:\ //" -e 's/<[^>]*>//')
RUN cd /gst-build-1.16 && meson install --no-rebuild -C build/ && rm -rf /gst-build-1.16
ENV GI_TYPELIB_PATH=/usr/local/lib/aarch64-linux-gnu/girepository-1.0

RUN apt-get install -y python3-colcon-common-extensions
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash && colcon build"
SHELL ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash"]
