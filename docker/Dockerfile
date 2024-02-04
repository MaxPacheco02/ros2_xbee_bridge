FROM ros:humble-ros-core

# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build
# install ssh client and git
RUN apt-get update
RUN apt-get install -y openssh-client git

# download public key for github.com
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Install tools
RUN apt-get update &&  apt-get install -y \
    vim \
    nano \
    ranger \
    tmux \
    tmuxinator

# Copy tools configuration
COPY ./to_copy/nanorc /root/.nanorc
COPY ./to_copy/ranger /root/.config/ranger/rc.conf
COPY ./to_copy/aliases /root/.bash_aliases
COPY ./to_copy/tmux /root/.tmux.conf
WORKDIR /root

RUN mkdir -p ws/src 
WORKDIR /root/ws/src

RUN --mount=type=ssh git clone git@github.com:mkrizmancic/ros2_xbee_bridge.git

RUN apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    python3-pip

RUN python3 -m pip install pyyaml roslibpy digi-xbee

WORKDIR /root/ws

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build"

# # Add separate aliases for noetic and ROS2_DISTRO
RUN echo "alias source_ros2='source /opt/ros/$ROS_DISTRO/setup.bash'" >> /root/.bashrc

# Add autocomplete
RUN echo "\
if ! shopt -oq posix; then \n\
  if [ -f /usr/share/bash-completion/bash_completion ]; then \n\
    . /usr/share/bash-completion/bash_completion \n\
  elif [ -f /etc/bash_completion ]; then \n\
    . /etc/bash_completion \n\
  fi \n\
fi" >> $HOME/.bashrc

RUN echo "alias source_ws='source /root/ws/install/setup.bash'" >> /root/.bashrc

WORKDIR /root/
CMD /bin/bash
