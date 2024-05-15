#!/bin/bash

APP_PATH=$HOME/.local/share/JetBrains/Toolbox/apps

PURPLE='\033[0;35m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
LIGHT_GREEN='\033[1;32m'
NC='\033[0m' # No Color

echo_info() {
  echo -e $PURPLE$1$NC
}

echo_success() {
  echo -e $LIGHT_GREEN$1$NC
}

echo_warning() {
  echo -e $YELLOW$1$NC
}

echo_failure() {
  echo -e $RED$1$NC
}

print_divider() {
  TITLE=$(echo "$1" | tr [:lower:] [:upper:])
  STATUS=$(echo "$2" | tr [:lower:] [:upper:])

  if [ $STATUS == FINISHED ]; then
    printf "$LIGHT_GREEN─%.0s$NC"  $(seq 1 103)
    printf "\n"
    printf "$LIGHT_GREEN%-90s : %10s$NC\n" "$TITLE" "$STATUS"
  else
    printf "$PURPLE%-90s : %10s$NC\n" "$TITLE" "$STATUS"
    printf "$PURPLE─%.0s$NC"  $(seq 1 103)
    printf "\n"
  fi
}

install_ros() {
  print_divider "Installing ROS packages" started

  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  sudo apt-get update

  if [ $(lsb_release -sc) == focal ]; then
    ROS_DISTRO=noetic
  elif [ $(lsb_release -sc) == bionic ]; then
    ROS_DISTRO=melodic
  else
    echo_failure "Only Ubuntu 20.04 and 18.04 are supported for ROS installation"
    return
  fi

  sudo apt-get install -y ros-$ROS_DISTRO-desktop ros-$ROS_DISTRO-rosmon ros-$ROS_DISTRO-behaviortree-cpp-v3 \
    ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-std-srvs \
    ros-$ROS_DISTRO-trac-ik-lib ros-$ROS_DISTRO-eigen-conversions ros-$ROS_DISTRO-rosbridge-suite \
    ros-$ROS_DISTRO-libfranka ros-$ROS_DISTRO-franka-gripper ros-$ROS_DISTRO-franka-hw \
    ros-$ROS_DISTRO-qb-device ros-$ROS_DISTRO-qb-hand

  print_divider "Successfully installed ROS $ROS_DISTRO packages" finished

  # Create ROS workspace
  print_divider "Set up ROS workspace" started

  source /opt/ros/$ROS_DISTRO/setup.bash

  if [ -d $HOME/catkin_ws ]; then
    echo_info "Found default ROS workspace $HOME/catkin_ws"
  else
    echo_info "Creating ROS workspace $HOME/catkin_ws ..."
    mkdir -p $HOME/catkin_ws/src
    echo_success "catkin_ws built"
  fi

  if [ ! -d $HOME/RoTools ]; then
    echo_failure "RoTools should be stored in the $HOME folder!"
  fi

  cd $HOME/RoTools && ./make_symlink.sh
  print_divider "Successfully set up ROS workspace" finished
}

install_frankx() {
  print_divider "Installing frankx packages" started

  if [ -d $HOME/frankx ]; then
    echo_info "frankx folder already exist in $HOME"
    cd $HOME/frankx && git checkout main
    git pull
  else
    echo_info "Cloning frankx to $HOME ..."
    cd $HOME
    git clone https://github.com/clover-cuhk/frankx.git
    cd $HOME/frankx && git checkout main
  fi

  mkdir build && cd build
  cmake ..
  make -j4
  sudo make install

  print_divider "Successfully installed frankx" finished
}

install_cartesio() {
  print_divider "Installing CartesI/O ..." started

  if [ $(lsb_release -sc) == focal ]; then
    LINK=https://github.com/ADVRHumanoids/XBotControl/releases/download/2.0-devel-core-updated/focal-latest.tar.gz
  elif [ $(lsb_release -sc) == bionic ]; then
    LINK=https://github.com/ADVRHumanoids/XBotControl/releases/download/2.0-devel-core-updated/bionic-latest.tar.xz
  else
    echo_failure "Only Ubuntu 20.04 and 18.04 are supported for CartesI/O installation"
    return
  fi

  cd ~/Downloads
  wget -cO cartesio.tar.gz $LINK
  tar -xzf cartesio.tar.gz
  DIR=$(find ./$(lsb_release -sc)-* -maxdepth 2 -type d -name $(lsb_release -sc)-\* -print | head -n1)
  cd $DIR
  echo $PWD
  ./install.sh

  grep -q "CartesI/O" $HOME/.bashrc
  if [ $? -ne 0 ]; then
    echo "" >>~/.bashrc
    echo "# CartesI/O" >>~/.bashrc
    echo "source /opt/xbot/setup.sh" >>~/.bashrc
  fi

  print_divider "Successfully installed CartesI/O $(lsb_release -sc)" finished
}

install_hpp() {
  print_divider "Installing Humanoid Path Planner ..." started

  sources $HOME/.bashrc
  if [ $(lsb_release -sc) == focal ]; then
    PYVER=38
    PYCODE=3.8
  elif [ $(lsb_release -sc) == bionic ]; then
    PYVER=36
    PYCODE=3.6
  else
    echo_failure "Only Ubuntu 20.04 and 18.04 are supported for HPP installation"
    return
  fi

  sudo sh -c 'echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg" > /etc/apt/sources.list.d/robotpkg.list'
  curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
  sudo apt-get update
  sudo apt-get install -y robotpkg-py${PYVER}-hpp-manipulation-corba robotpkg-py${PYVER}-qt5-hpp-gepetto-viewer \
    robotpkg-py${PYVER}-hpp-tutorial robotpkg-py${PYVER}-qt5-hpp-gui robotpkg-py${PYVER}-qt5-hpp-plot \
    robotpkg-py${PYVER}-hpp-environments robotpkg-py${PYVER}-eigenpy robotpkg-hpp-fcl

  grep -q "HPP" $HOME/.bashrc
  if [ $? -ne 0 ]; then
    echo "" >>~/.bashrc
    echo "# HPP" >>~/.bashrc
    echo "export PATH=/opt/openrobots/bin:\$PATH" >>~/.bashrc
    echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH" >>~/.bashrc
    echo "export PYTHONPATH=/opt/openrobots/lib/python$PYCODE/site-packages:\$PYTHONPATH" >>~/.bashrc
    echo "export ROS_PACKAGE_PATH=/opt/openrobots/share:\$ROS_PACKAGE_PATH" >>~/.bashrc
    echo "export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH" >>~/.bashrc
    echo "export PKG_CONFIG_PATH=/opt/openrobots:\$PKG_CONFIG_PATH" >>~/.bashrc
  fi

  print_divider "Successfully installed Humanoid Path Planner for Python $PYCODE" finished
}

install_pinocchio() {
  print_divider "Installing pinocchio ..." started

  sources $HOME/.bashrc
  if [ $(lsb_release -sc) == focal ]; then
    PYCODE=3.8
  elif [ $(lsb_release -sc) == bionic ]; then
    PYCODE=3.6
  else
    echo_failure "Only Ubuntu 20.04 and 18.04 are supported for Pinocchio installation"
    return
  fi

  cd ~
  git clone --recursive https://github.com/stack-of-tasks/pinocchio
  cd pinocchio/ && git checkout master
  mkdir build && cd build
  cmake -DBUILD_WITH_COLLISION_SUPPORT=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
  make -j4
  sudo make install

  grep -q "pinocchio" $HOME/.bashrc
  if [ $? -ne 0 ]; then
    echo "" >>~/.bashrc
    echo "# pinocchio" >>~/.bashrc
    echo "export PATH=/usr/local/bin:\$PATH" >>~/.bashrc
    echo "export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:\$PKG_CONFIG_PATH" >>~/.bashrc
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >>~/.bashrc
    echo "export PYTHONPATH=/usr/local/lib/python$PYCODE/site-packages:\$PYTHONPATH" >>~/.bashrc
    echo "export CMAKE_PREFIX_PATH=/usr/local:\$CMAKE_PREFIX_PATH" >>~/.bashrc
  fi

  print_divider "Successfully installed pinocchio" finished
}

install_ocs2() {
  print_divider "Installing OCS2 ..." started

  sources $HOME/.bashrc
  if [ $(lsb_release -sc) == focal ]; then
    PYCODE=3.8
  else
    echo_failure "Only Ubuntu 20.04 is supported for OCS2 installation"
    return
  fi

  sudo apt-get update
  sudo apt-get install -y libglpk-dev ros-noetic-pybind11-catkin python3-catkin-tools doxygen doxygen-latex \
    liburdfdom-dev liboctomap-dev libassimp-dev ros-noetic-rqt-multiplot python3-sphinx checkinstall ros-noetic-grid-map-msgs

  cd ~
  git clone https://github.com/raisimTech/raisimLib.git
  cd raisimLib/
  mkdir build && cd build
  cmake .. -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
  make -j4

  cd ~/catkin_ws/src/
  git clone https://github.com/leggedrobotics/ocs2.git
  git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
  cd ~/catkin_ws
  catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo

  print_divider "Successfully installed OCS2" finished
}

install_python() {
  print_divider "Installing Python packages" started
  sudo apt-get install patchelf

  pip install -U mujoco imageio

  grep -q "RoTools/src" $HOME/.bashrc
  if [ $? -ne 0 ]; then
    echo_info "Adding $HOME/RoTools/src to PYTHONPATH ..."
    echo "" >> ~/.bashrc
    echo "# RoTools" >> ~/.bashrc
    echo "export PYTHONPATH=$HOME/RoTools/src:\$PYTHONPATH" >> ~/.bashrc
    echo_success "Successfully add $HOME/RoTools/src to PYTHONPATH"
  else
    echo_info "$HOME/RoTools/src has already been added to PYTHONPATH"
  fi
  print_divider "Python packages installed" finished
}

install_sublime_text() {
  print_divider "Installing Sublime Text ..." started
  wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
  echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
  sudo apt-get update
  sudo apt-get install -y sublime-text
  print_divider "Successfully installed Sublime Text" finished
}

install_jetbrains_toolbox() {
  print_divider "Installing JetBrains Toolbox ..." started
  cd ~/Downloads
  wget -cO jetbrains-toolbox.tar.gz "https://data.services.jetbrains.com/products/download?platform=linux&code=TBA"
  tar -xzf jetbrains-toolbox.tar.gz
  DIR=$(find . -maxdepth 1 -type d -name jetbrains-toolbox-\* -print | head -n1)
  cd $DIR
  ./jetbrains-toolbox
  cd ..
  rm -r $DIR
  rm jetbrains-toolbox.tar.gz
  print_divider "Successfully installed JetBrains Toolbox" finished
}

help() {
  # Display Help
  echo "===== RoTools Setup Tools Usage Guide ====="
  echo "Syntax: setup_tools.sh [option]"
  echo "options:"
  echo "no option    Install all packages, recommended for new machine."
  echo "-h | --help  Print this Help."
  echo "--ros        Install ROS only."
  echo "--cio        Install CartesIO only."
  echo "--hpp        Install HPP only."
  echo "--pin        Install pinocchio only."
  echo "--ocs2       Install OCS2 only."
  echo "--py         Install Python packages only."
  echo "--sub        Install Sublime Text only."
  echo "--jet        Install JetBrains Toolbox only."
}

if [ $# -eq 1 ]; then
  case "$1" in
  -h | --help)
    help
    exit
    ;;
  *) # Pass
    ;;
  esac
fi

sudo apt-get update
sudo apt-get install -y wget apt-transport-https libmatio-dev screen

if [ $# -eq 0 ]; then
  install_ros
  install_frankx
  install_cartesio
  install_hpp
  install_pinocchio
#  install_ocs2  # Temporary suspend this due to comparability error
  install_python
  install_sublime_text
  install_jetbrains_toolbox
else
  case "$1" in
  -h | --help)
    help
    exit
    ;;
  --ros)
    install_ros
    exit
    ;;
  --cio)
    install_cartesio
    exit
    ;;
  --frx)
    install_frankx
    exit
    ;;
  --hpp)
    install_hpp
    exit
    ;;
  --pin)
    install_hpp
    install_pinocchio
    exit
    ;;
  --ocs2)
    install_hpp
    install_pinocchio
    install_ocs2
    exit
    ;;
  --py)
    install_python
    exit
    ;;
  --sub)
    install_sublime_text
    exit
    ;;
  --jet)
    install_jetbrains_toolbox
    exit
    ;;
  *) # Invalid option
    help
    exit
    ;;
  esac
fi

exit 0
