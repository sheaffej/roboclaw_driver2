FROM ros:galactic-ros-base

ENV ROS_WS /ros2
ENV PKG_NAME roboclaw_driver
ENV COLCON_BUILD_OPTS "--symlink-install"
ENV PYTEST_ADDOPTS "--color=yes"

# SHELL [ "bash", "-c"]

# Install pip and system packages
RUN apt update \
	&& apt install -y \
        python-is-python3 \
        python3-pip \
        vim \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# Install Python testing packages
COPY requirements-dev.txt .
RUN pip install -r requirements-dev.txt

# RUN echo "source /ros_entrypoint.sh" >> /root/.bashrc && \
# 	echo "source /root/.bashrc" >> /root/.bash_profile
RUN echo \
'alias wsenv="source /opt/ros/${ROS_DISTRO}/setup.bash && source ${ROS_WS}/install/setup.bash"' \
>> /root/.bashrc


RUN mkdir -p ${ROS_WS}/src

# Add this package
RUN ln -s /workspaces/${PKG_NAME} ${ROS_WS}/src/${PKG_NAME}

# Add other source packages
RUN cd ${ROS_WS}/src \
&& git clone https://github.com/sheaffej/roboclaw_msgs.git

# Install dependencies
RUN cd ${ROS_WS} \
&& apt update \
&& rosdep update \
&& rosdep install --from-paths src -y \
&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# # Build the workspace
# RUN source "/opt/ros/$ROS_DISTRO/setup.bash" \
# && cd $ROS_WS \
# && colcon build ${COLCON_BUILD_OPTS} \
# && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
