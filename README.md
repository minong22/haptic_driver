# haptic_driver
haptic device omega.6 ros driver
You can get the pose of the haptic device and set the force of the device.       

reference : (https://github.com/jacknlliu/haptic_ros_driver) 

## 0.prerequisities
libsusb-1.0 is needed to connect haptic device and computer.           
And you have to designate the usb's path.

## 1. How to use
do clone repository overall codes.         

Before start the ros driver, you have to check the haptic device. 

```linux
cd catkin_ws/src/sdk-3.14.0/bin
sudo ./HapticInit
```

after that you can detect your haptic device and initialize it.

Then you are ready to start the haptic_ros_driver

```linux
cd catkin_ws/
catkin_make
rosrun haptic_ros_driver haptic_ros_driver
```
## 2. Introduction
By using this code, you can publish the position & orientaton data and you can subscribe the force data.       
And there are rqt_pub which i use it to get the graph of force simultaneously.       
* publisher
  * pos        
  topic name : ```/haptic/position```        
  type : ```geometry_msgs::Vector3Stamped```
  
  * ori          
  topic name : ```/haptic/ori```        
  type : ```geometry_msgs::Vector3```
  
  * pose        
  topic name : ```/haptic/pose```         
  type : ```geometry_msgs::Pose```
  
  * rqt_force        
  topic name : ```/rqt/force```         
  type : ```geometry_msgs::Vector3```

* subscriber
  * force        
  topic name : ```/haptic/force```          
  type : ```geometry_msgs::Vector3```          
         
And if you need mapping the force, use the ForceMapping function inside of the code !        
dhd library which force dimension is served makes easy to operate the device.        
I use dhdGetPosition, dhdGetOrientationRad, dhdSetForce function. 
         

### Enjoy your operation !
