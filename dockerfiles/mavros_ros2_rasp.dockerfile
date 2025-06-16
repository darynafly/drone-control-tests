FROM ros:humble-ros-base

# Trick to get apt-get to not prompt for timezone in tzdata
ENV DEBIAN_FRONTEND=noninteractive

ARG STARTDELAY=5
ENV STARTDELAY=$STARTDELAY

# Install MAVROS and some other dependencies for later
RUN apt-get update && apt-get install -y ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs wget

# Dependency from https://github.com/mavlink/mavros/blob/master/mavros/README.md
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh

RUN /bin/bash -c 'echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc'

ENTRYPOINT ["/bin/bash"]
