# Monocular odometry
This project is aimed at indoor robot navigation system. It requires only one camera looking parallel to ground plane. 

## Description 
This algorithm uses combining of differential epipolar constraint and optical flow estimation.  

## Prerequisites 
* c++11
* OpenCV 3
* ROS Kinetic

## ToDo
* Calibrate a camera:
  * Find focus
  * Pixel size
  * Height (distance between ground plane and camera center location)
  
 ## Notes
 * The floor must be textured enough. It most likely may not work on monotonous pictured floor.
