# slam_simulator
A slam simulator in a ROS environment for simulating basic slam algorithms such as ukf, ekf, etc.

The project use ukf and ekf now!

To watch the video by using ukf, please click on the picture below!
[![Watch the video](https://github.com/plumewind/slam_simulator/blob/master/view.png)](https://youtu.be/lP2fqqZU4sg)

To watch the video by using ekf, please click on the picture below!
[![Watch the video](https://github.com/plumewind/slam_simulator/blob/ekf/ekf_slam.png)](https://www.youtube.com/watch?v=554A07SMJjU)


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
  

