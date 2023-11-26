ARG ROS_DISTRO=humble
ARG DEBIAN_FRONTEND=noninteractive

FROM public.ecr.aws/docker/library/ros:$ROS_DISTRO

# Directory where the image-specific files are located
ARG IMAGE_DIR

ENV AMENT_PREFIX_PATH=/opt/ros/$ROS_DISTRO
ENV COLCON_PREFIX_PATH=/opt/ros/$ROS_DISTRO
ENV LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib
ENV PATH=/opt/ros/$ROS_DISTRO/bin:$PATH
ENV PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# System dependencies
RUN apt-get update && apt-get install -y \
    bash-completion cmake vim build-essential \
    apt-utils zip unzip \
    wget iputils-ping openssh-client net-tools \
    linux-tools-generic lttng-tools liblttng-ust-dev \
    python3-lttng python3-pip python3-venv \
    device-tree-compiler \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
COPY $IMAGE_DIR/files/requirements.txt requirements.txt
RUN python3 -m pip install -r requirements.txt && rm requirements.txt

# ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-rviz2

# Install SSH Server
RUN apt-get update && apt-get --yes --no-install-recommends install \
    openssh-server && \
    mkdir -p /var/run/sshd && \
    chmod 0755 /var/run/sshd && \
    echo "StrictHostKeyChecking no" >> /etc/ssh/ssh_config && \
    sed -i 's/AcceptEnv/# AcceptEnv/' /etc/ssh/sshd_config

# ROS 2 acceleration firmware dependencies
RUN apt-get update && apt-get install -y \
    gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
    libtinfo5 locales

# micro-ROS dependencies
RUN pip3 install catkin_pkg lark-parser empy colcon-common-extensions

# Configure locales
ENV LANG=en_US.UTF-8
RUN apt-get install -y locales && \
    echo "$LANG UTF-8" > /etc/locale.gen && \
    dpkg-reconfigure --frontend=noninteractive locales && \
    update-locale LANG=$LANG

# Create and configure non-root user
ARG USERNAME=ros
ENV USERNAME=${USERNAME}

RUN echo "Creating $USERNAME user" && \
    useradd --create-home -s /bin/bash $USERNAME && \
    mkdir -p /home/$USERNAME/.ssh && \
    \
    echo "Changing $USERNAME user password" && \
    echo -n "$USERNAME:$USERNAME" | chpasswd && \
    \
    echo "Enable passwordless sudo for the $USERNAME user" && \
    echo "$USERNAME ALL=NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \
    \
    echo "Configuring ssh keys" && \
    echo "ssh-rsa AAAAB3NzaC1yc2EAAAABIwAAAQEA6NF8iallvQVp22WDkTkyrtvp9eWW6A8YVr+kz4TjGYe7gHzIw+niNltGEFHzD8+v1I2YJ6oXevct1YeS0o9HZyN1Q9qgCgzUFtdOKLv6IedplqoPkcmF0aYet2PkEDo3MlTBckFXPITAMzF8dJSIFo9D8HfdOV0IAdx4O7PtixWKn5y2hMNG0zQPyUecp4pzC6kivAIhyfHilFR61RGL+GPXQ2MWZWFYbAGjyiYJnAmCP3NOTd0jMZEnDkbUvxhMmBYSdETk1rRgm+R4LOzFUGaHqHDLKLX+FIPKcF96hrucXzcWyLbIbEgE98OHlnVYCzRdK8jlqm8tehUc9c9WhQ== Vagrant insecure public key" > /home/$USERNAME/.ssh/authorized_keys && \
    chown -R $USERNAME: /home/$USERNAME/.ssh && \
    ssh-keyscan -H github.com > /home/$USERNAME/.ssh/known_hosts && \
    \
    echo "Setting correct permissions to the workspace" && \
    mkdir -p /home/$USERNAME/ws && chown -R $USERNAME: /home/$USERNAME/ws && \
    \
    echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Configure user groups
RUN sudo usermod -aG video $USERNAME
RUN sudo usermod -aG tracing $USERNAME

# Update MOTD info
RUN rm /etc/update-motd.d/10-help-text /etc/update-motd.d/50-motd-news /etc/update-motd.d/60-unminimize
COPY $IMAGE_DIR/version /etc/env_version
COPY $IMAGE_DIR/files/motd/static /etc/update-motd.d/static
COPY $IMAGE_DIR/files/motd/version /etc/update-motd.d/10-version

# Launch SSH daemon
EXPOSE 22
CMD /usr/sbin/sshd -D -o UseDNS=no -o AcceptEnv=DISPLAY