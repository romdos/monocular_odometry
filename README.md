# Monocular odometry
This project is aimed at indoor robot navigation system. It requires only one camera looking parallel to ground plane. 
The algorithm used in this project combines differential epipolar constraint (D. J. Heeger et al.) and optical flow estimation.  See also 
* David J. Heeger, Allan D. Jepson "_Subspace methods for recovering rigid motion I: Algorithm and implementation_", International Journal of Computer Vision, 1992

## Prerequisites 
* c++11
* OpenCV 3
* ROS Kinetic

## Build
It is a ROS package, so build it downloading into catkin workspace and running catkin_make.
    
## ToDo
* Calibrate a camera:
  * Find focus
  * Pixel size
  * Height (distance between ground plane and camera center location)
  
 ## Notes
 * The floor must be textured enough. It most likely may not work on monotonous pictured floor.
