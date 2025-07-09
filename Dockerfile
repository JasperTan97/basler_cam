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

USER $USERNAME
RUN rosdep update 

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
CMD [ "/bin/bash" ]
WORKDIR /ros2_ws
RUN source ~/.bashrc
