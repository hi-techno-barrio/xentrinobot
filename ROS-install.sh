
#!/bin/bash

set -e

echo " Hi-Techno Barrio"
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

  buster)
    echo -n "ros-melodic"
    #ROSDISTRO=melodic
    export ROS_DISTRO=melodic
    ;;
  *)
    echo -n "No ROS version for your debian package"
    ;;
esac

   echo Installing ros-$ROS_DISTRO
   sudo apt-get update

   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt-get update   
   sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential  cmake 
 
   sudo rosdep init   
   rosdep update  
     
                                                                                                                                    4,0-1         Top
#Install ROS Distro
   echo "Installing ROS-$ROS_DISTRO Full Desktop Version"

   rosinstall_generator desktop --rosdistro $ROS_DISTRO --deps --wet-only --tar > $ROS_DISTRO-desktop-wet.rosinstall
   wstool init -j8 src $ROS_DISTRO-desktop-wet.rosinstall
   rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

   mkdir -p $HOME_DIR/xentrinobot_ws/src
   cd $HOME_DIR/xentrinobot_ws/
 
   sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/$ROS_DISTRO -j2

   mkdir -p external_src
   cd external_src
      wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
      unzip assimp-3.1.1_no_test_models.zip
   cd assimp-3.1.1
       cmake
       make
       sudo make install

# Check for installed ROS distro
   source /opt/ros/$ROS_DISTRO/setup.bash
   echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
   source ~/.bashrc


echo ""
echo "ROS $(rosversion -d) Successful Instalation!"




