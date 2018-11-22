# slam_simulator
A slam simulator in a ROS environment for simulating basic slam algorithms such as ukf, ekf, etc.

To watch the video, please click on the picture below!
[![Watch the video](https://github.com/plumewind/slam_simulator/blob/master/view.png)](https://youtu.be/lP2fqqZU4sg)

How to use ?
------------------
Open an Ubuntu terminal and create a workspace. Of course, you must have already installed the ROS system:

  mkdir ~/catkin_ws/src
  
  cd ~/catkin_ws/src
  
  git clone https://github.com/plumewind/slam_simulator.git
  
  cd ..
  
  catkin_make
  
  source devel/setup.bash
  
  roslaunch slam_simulator ukfslam_simulator.launch
  
