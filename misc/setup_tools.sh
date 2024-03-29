#!/bin/bash

APP_PATH=$HOME/.local/share/JetBrains/Toolbox/apps

RED='\033[0;31m'
YELLOW='\033[1;33m'
LIGHT_GREEN='\033[1;32m'
NC='\033[0m' # No Color

echo_success() {
  echo -e $LIGHT_GREEN$1$NC
}

echo_warning() {
  echo -e $YELLOW$1$NC
}

echo_failure() {
  echo -e $RED$1$NC
}

install_ros() {
  echo_warning "Installing ROS ..."

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
    ros-$ROS_DISTRO-trac-ik-lib ros-$ROS_DISTRO-eigen-conversions ros-$ROS_DISTRO-rosbridge-suite

  echo_success "Successfully installed ROS $ROS_DISTRO packages"

  # TODO Create ROS workspace
  source /opt/ros/$ROS_DISTRO/setup.bash
}

install_cartesio() {
  echo_warning "Installing CartesI/O ..."

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

  echo "" >>~/.bashrc
  echo "# CartesI/O" >>~/.bashrc
  echo "source /opt/xbot/setup.sh" >>~/.bashrc

  echo_success "Successfully installed CartesI/O $(lsb_release -sc)"
}

install_hpp() {
  echo_warning "Installing Humanoid Path Planner ..."

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

  echo "" >>~/.bashrc
  echo "# HPP" >>~/.bashrc
  echo "export PATH=/opt/openrobots/bin:\$PATH" >>~/.bashrc
  echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH" >>~/.bashrc
  echo "export PYTHONPATH=/opt/openrobots/lib/python$PYCODE/site-packages:\$PYTHONPATH" >>~/.bashrc
  echo "export ROS_PACKAGE_PATH=/opt/openrobots/share:\$ROS_PACKAGE_PATH" >>~/.bashrc
  echo "export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH" >>~/.bashrc
  echo "export PKG_CONFIG_PATH=/opt/openrobots:\$PKG_CONFIG_PATH" >>~/.bashrc

  echo_success "Successfully installed Humanoid Path Planner for Python $PYCODE"
}

install_pinocchio() {
  echo_warning "Installing pinocchio ..."

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

  echo "" >>~/.bashrc
  echo "# pinocchio" >>~/.bashrc
  echo "export PATH=/usr/local/bin:\$PATH" >>~/.bashrc
  echo "export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:\$PKG_CONFIG_PATH" >>~/.bashrc
  echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >>~/.bashrc
  echo "export PYTHONPATH=/usr/local/lib/python$PYCODE/site-packages:\$PYTHONPATH" >>~/.bashrc
  echo "export CMAKE_PREFIX_PATH=/usr/local:\$CMAKE_PREFIX_PATH" >>~/.bashrc

  echo_success "Successfully installed pinocchio"
}

install_ocs2() {
  echo_warning "Installing OCS2 ..."

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
  # Rename the pkg to 'raisim' when checkinstall
  make -j4 && sudo checkinstall

  cd ~/catkin_ws/src/
  git clone https://github.com/leggedrobotics/ocs2.git
  git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
  cd ~/catkin_ws
  catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo

  echo_success "Successfully installed OCS2"
}

install_sublime_text() {
  echo_warning "Installing Sublime Text ..."
  wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
  echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
  sudo apt-get update
  sudo apt-get install -y sublime-text
  echo_success "Successfully installed Sublime Text"
}

install_jetbrains_toolbox() {
  echo_warning "Installing JetBrains Toolbox ..."
  cd ~/Downloads
  wget -cO jetbrains-toolbox.tar.gz "https://data.services.jetbrains.com/products/download?platform=linux&code=TBA"
  tar -xzf jetbrains-toolbox.tar.gz
  DIR=$(find . -maxdepth 1 -type d -name jetbrains-toolbox-\* -print | head -n1)
  cd $DIR
  ./jetbrains-toolbox
  cd ..
  rm -r $DIR
  rm jetbrains-toolbox.tar.gz
  echo_success "Successfully installed JetBrains Toolbox"
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
  echo "--sub        Install Sublime Text only."
  echo "--jet        Install JetBrains Toolbox only."
}

sudo apt-get update
sudo apt-get install -y wget apt-transport-https libmatio-dev screen

if [ $# -eq 0 ]; then
  install_ros
  install_cartesio
  install_hpp
  install_pinocchio
  install_ocs2
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
