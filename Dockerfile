# Start with an official ROS 2 base image for the desired distribution
FROM ros:humble-ros-base

ARG USER_UID=1002
ARG USER_GID=1003
ARG USERNAME=user

# Install essential packages and ROS development tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        iputils-ping \
        libgl1-mesa-glx \
        nano \
        openssh-client \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        python3-pip \
        python3.10-venv \
        ros-humble-cv-bridge \
        sudo \
        vim \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 dependencies here
# RUN sudo apt-get update \
#     && sudo apt-get install -y --no-install-recommends \
#     && sudo apt-get clean \
#     && sudo rm -rf /var/lib/apt/lists/*

RUN rosdep update 

COPY requirements.txt /ros2_ws/requirements.txt

RUN pip3 install --no-cache-dir -r /ros2_ws/requirements.txt

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
&& useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
&& echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
&& echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
&& echo "source /ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
&& echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc \
&& echo "ROS_DOMAIN_ID=27" >> /home/$USERNAME/.bashrc

COPY ./pylon_setup/pylon-25.06.4_linux-x86_64_debs.tar.gz /tmp/
RUN tar -xzf /tmp/pylon-25.06.4_linux-x86_64_debs.tar.gz -C /tmp \
    && rm /tmp/pylon-25.06.4_linux-x86_64_debs.tar.gz
RUN chmod 1777 /tmp
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libgl1-mesa-glx \
        libxcb-xinerama0 \
        libxcb-xinput0 \
        libxcb-cursor0 \
        libgl1 \
        libglvnd0 \
        libglx-mesa0 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update \
    && dpkg -i /tmp/pylon_*.deb \
    && apt-get install -f -y \
    && rm -rf /var/lib/apt/lists/* /tmp/pylon_*.deb

USER $USERNAME
WORKDIR /ros2_ws

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
CMD [ "/bin/bash" ]
RUN source ~/.bashrc
