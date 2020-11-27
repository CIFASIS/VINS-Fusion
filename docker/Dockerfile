FROM ros:kinetic-perception

ENV CERES_VERSION="1.12.0"

      # set up thread number for building
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      apt-get update && apt-get install -y \
      cmake \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python-catkin-tools \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf && \
      rm -rf /var/lib/apt/lists/* && \
      # Build and install Ceres
      git clone https://ceres-solver.googlesource.com/ceres-solver && \
      cd ceres-solver && \
      git checkout tags/${CERES_VERSION} && \
      mkdir build && cd build && \
      cmake .. && \
      make -j$(USE_PROC) install && \
      rm -rf ../../ceres-solver

# if you want to reference a previously defined env variable in another definition, use multiple ENV
ENV CATKIN_WS=/root/catkin_ws VINS_ROOT=/root/catkin_ws/src/VINS-Fusion/

# Copy VINS-Fusion
COPY ./ $CATKIN_WS/src/VINS-Fusion/

WORKDIR $CATKIN_WS
ENV TERM=xterm PYTHONIOENCODING=UTF-8

# Build VINS-Fusion
COPY ./scripts/ $CATKIN_WS
RUN ["/bin/bash", "-c", "chmod +x build.sh && chmod +x modify_entrypoint.sh && sync && ./build.sh && ./modify_entrypoint.sh"]
