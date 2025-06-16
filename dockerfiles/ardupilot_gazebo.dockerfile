FROM ardupilot/ardupilot-dev-ros AS main-setup

# Install required packages
RUN apt update && \
    apt upgrade -y && \
    apt install -y lsb-release gnupg curl sudo default-jre && \
    curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    sudo apt update && \
    sudo apt install -y gz-harmonic && \
    apt install -y ros-humble-ament-cmake-mypy

# Prepare workspace
RUN mkdir -p /home/ardupilot/ros2_ws/src && \
    cd /home/ardupilot/ros2_ws/src && \
    vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos

# Install dependencies
RUN cd /home/ardupilot/ros2_ws/ && \
    rosdep update && \
    rosdep install --from-paths /home/ardupilot/ros2_ws/src --ignore-src -r -y

# Build Micro-XRCE-DDS-Gen
RUN cd /home/ardupilot/ros2_ws && \
    git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git && \
    cd Micro-XRCE-DDS-Gen && \
    ./gradlew assemble

# Build ROS 2 workspace
RUN cd /home/ardupilot/ros2_ws && \
    /bin/bash -c 'source /opt/ros/humble/setup.bash; colcon build --packages-up-to ardupilot_dds_tests'

# Import and build additional repositories
RUN cd /home/ardupilot/ros2_ws && \
    vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src && \
    export GZ_VERSION=harmonic && \
    # source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    /bin/bash -c 'source /opt/ros/humble/setup.bash; colcon build --packages-up-to ardupilot_gz_bringup --parallel-workers 2'

RUN apt update && apt-get install -y libignition-gazebo6-dev

RUN chmod +x /root/run.sh

RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && cd /home/ardupilot/ros2_ws && colcon build --packages-up-to ardupilot_gz_bringup'
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && cd /home/ardupilot/ros2_ws && colcon build --packages-select yolov8_bringup yolov8_msgs yolov8_ros'

RUN apt install -y ros-humble-mavros ros-humble-mavros-extras
RUN apt remove modemmanager && systemctl disable ModemManager # conflicts with apm

# Set up environment
RUN chmod +x /root/run.sh && \
    echo "export PATH=$PATH:/home/ardupilot/Tools/autotest:/usr/lib/ccache:/usr/local/bin/waf" >> /root/.bashrc \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /home/ardupilot/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export DISPLAY=:1.0" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=5" >> /root/.bashrc && \
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /root/.bashrc && \
    echo "export GZ_VERSION=harmonic" >> /root/.bashrc

ENTRYPOINT ["/bin/bash"]
