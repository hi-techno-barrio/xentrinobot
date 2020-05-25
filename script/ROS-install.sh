#!/bin/bash

set -e
echo "#####################################################################"
echo "#                Hi-Techno Barrio                                   #" 
echo "#                ROS Philippines                                    #"
echo "#####################################################################"


echo "We are about to install ROS 4 Pi"
HOMEDIR=$(pwd)
CPU_ARCH=$(uname -i)
DEBIAN_VERSION=$(lsb_release -c -s)

echo -n "Your Debian version  is: " 

case $DEBIAN_VERSION in

  jessie )
    echo -n "ros-kinetic"
    #ROSDISTRO=kinetic
    export ROS_DISTRO=kinetic
    ;;
   wheezy )
    echo -n "ros-indigo"
    #ROSDISTRO=kinetic
    export ROS_DISTRO=indigo
    ;;
  buster)
    echo -n "ros-melodic"
    #ROSDISTRO=melodic
    export ROS_DISTRO=melodic
    ;;
  *)
    echo -n "No ROS version for your debian package"
    ;;
esac
echo
echo "#####################################################################"
echo "#     "installing ROS repositories"                                 #"
echo "#####################################################################"

   sudo apt update  
   sudo apt-get install dirmngr
   sudo apt-get install  libogre-1.9-dev  

  echo "Installing ros-$ROS_DISTRO  package"
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 
  sudo apt-get  -y update
  
 echo "installing  ROS bootstrap"
    sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential  cmake
 
 echo "initializing rosdep"
    sudo rosdep init  
    rosdep update

echo "#####################################################################"
echo "#              create Workspace"                                    #"
echo "#####################################################################"
    cd $HOME_DIR
    mkdir -p ~/xentrinobot_ws/src
    cd ~/xentrinobot_ws/


echo "#####################################################################"
echo "#     "Installing ROS-$ROS_DISTRO Core Package"                     #"
echo "#####################################################################"
    #rosinstall_generator desktop --rosdistro $ROS_DISTRO --deps --wet-only --tar > $ROS_DISTRO-desktop-wet.rosinstall   
    #wstool init -j8 src $ROS_DISTRO-desktop-wet.rosinstall

    rosinstall_generator ros_comm --rosdistro $ROS_DISTRO --deps --wet-only --tar > $ROS_DISTRO-ros_comm-wet.rosinstall
    wstool init src $ROS_DISTRO-ros_comm-wet.rosinstall

echo "#####################################################################" 
echo "#     resolving ROS dependencies                                    #"
echo "#####################################################################"
      mkdir -p ~/xentrinobot_ws/external_src
      cd  ~/xentrinobot_ws/external_src
      wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
      unzip assimp-3.1.1_no_test_models.zip
      cd assimp-3.1.1
      cmake .
      make
      sudo make install

    cd  ~/xentrinobot_ws
      rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r --os=debian:$DEBIAN_RELEASE
     #rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:buster
  
echo "#####################################################################"
echo "#       ROS $(rosversion -d) Successful Instalation!                #"
echo "#####################################################################"  


echo "#####################################################################"
echo "#             Adding ROS libraries"                                 #"
echo "#####################################################################" 

   sudo apt-get update
   sudo apt-get install -y \
   avahi-daemon \
   openssh-server \
   python-setuptools \
   python-dev \
   build-essential \
   python-pyudev

   sudo apt-get install python-pip
   sudo python2.7 -m pip install -U platformio
   sudo rm -rf $HOME_DIR/.platformio/

echo "#####################################################################"
echo "#           building workspace catkin                               #"
echo "#####################################################################" 
    cd $HOME_DIR
    cd ~/xentrinobot_ws/   
     # sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j2
    sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/$ROS_DISTRO -j2

echo "#####################################################################"
echo "#          source the new installation                              #"
echo "#####################################################################" 
    #catkin_init_workspace
    source /opt/ros/$ROS_DISTRO/setup.bash
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    source ~/.bashrc 

































































