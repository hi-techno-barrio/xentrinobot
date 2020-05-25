


xentrinobot-ros 
is a modified version of linorobot firmware and refined base controller for ROS1 and ROS2 
the purpose is to give simple coding easy to follow by the learner and teachers.

![Xentrinobots ](https://raw.githubusercontent.com/hi-techno-barrio/XENTRINOBot-ROS/master/images/Xentrino-Set-Robots.png)

Thanks to:
ROS Community
ROS Philippines
Linorobot

References:
ROS wiki

Instructions:
Create a simple installation of ROS1/ROS2 for Debian Pi
sudo apt-get install ros-$ROS_DISTRO-openni-camera ros-$ROS_DISTRO-openni-launch
roslaunch pkg-name display.launch model:='$(find pkg-name)/xentrino.urdf'
