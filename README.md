


xentrinobot-ros 
is a general purpose multiple platforms mobile robotics  compliant to  ROS1 and ROS2, 
the purpose is to give lesser parts yet configurable to many setups of mobile robtics
simple ,robust,rugged affordable robotics platform for coding,debugging,porting using ROS  .

XetrinoBot Mecanum Version
![Xentrinobots ](https://github.com/hi-techno-barrio/XENTRINOBot-ROS/blob/master/images/Xentrino-4WD-Mechanum.png)

Xentrio 2WD Version
![Xentrinobots ](https://github.com/hi-techno-barrio/xentrinobot/blob/master/images/Xentrino-2WD.png)

XentrinoBot Skid Version
![Xentrinobots ](https://github.com/hi-techno-barrio/XENTRINOBot-ROS/blob/master/images/Xentrino-4WD.png)

XentrinoBot Set
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
