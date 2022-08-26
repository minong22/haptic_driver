# haptic_driver
haptic device omega.6 ros driver
reference : (https://github.com/jacknlliu/haptic_ros_driver) 

## 0.prerequisities
libsusb-1.0 is needed to connect haptic device and computer. 
And you have to designate the usb's path.

## 1. How to use
do clone repository overall codes.

* Before start the ros driver, you have to check the haptic device. 

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
