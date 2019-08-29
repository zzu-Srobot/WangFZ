# Index

[TOC]

# 安装依赖

`rosdep install --from-paths src --ignore-src -r -y`
+ This command magically installs all the packages that the packages in your catkin workspace depend upon but are missing on your computer.

# usb_cam安装

+ 通过`lsusb`查看摄像头对应的信息
+ 在[这里](http://www.ideasonboard.org/uvc/#devices)查找设备ID，如果ID在此列表则是uvc_cam，否则是usb_cam。已知Kinect2是usb_cam
+ ROS_WIKI：[A ROS Driver for V4L USB Cameras](http://wiki.ros.org/usb_cam)
 + Bug / feature tracker: https://github.com/bosch-ros-pkg/usb_cam/issues
 + Source: git https://github.com/ros-drivers/usb_cam.git (branch: develop)
[安装教程](https://www.jianshu.com/p/e0d96f55f307)
# iai_kinect
+ `kinect2`的`ros`软件包.该软件包发布了`kinect2`捕捉的图像话题
+ github链接：https://github.com/code-iai/iai_kinect2

  + [[DepthRegistrationOpenCL::init] could not find any suitable device](https://github.com/code-iai/iai_kinect2/issues/371)
  + 解决方案:
  ```sh
  Linux:$ roscore
  Linux:/catkin_ws$ rosrun kinect2_bridge kinect2_bridge_depth_method:=opengl _reg_method:=cpu
  Linux:~$ rosrun kinect2_viewer kinect2_viewer
  ```
  + 需要用`.launch`文件启动以后才会出现`/kinect2/×/Point`点云数据类型的话题
# body_pose
+ 基于`Tensorflow`定位`2d`图片当中的`人体骨骼点`
+ github链接: https://github.com/BluewhaleRobot/body_pose
  - 建议指令集在安装`Tensorflow`的虚拟环境`venv`中安装
  - 使用 `pip3`来安装软件依赖集当中的`scipy`，否则会报错
  - 使用`sudo -H python -m pip install pip --upgrade --force`来升级`pip`
# xiaoqiang_tracker
+ 使用`PID`算法,基于`body_pose`实现的跟随
+ github链接：https://github.com/bluewhalerobot/xiaoqiang_track
  - 教程链接：http://community.bwbot.org/topic/500/%E4%BD%BF%E7%94%A8xiaoqiang_track%E8%BF%9B%E8%A1%8C%E4%BA%BA%E4%BD%93%E8%B7%9F%E9%9A%8F%E5%92%8C%E8%BF%BD%E8%B8%AA/2
# xf-ros
+ 集成自科大讯飞DEMO的ros软件包
  - github链接:https://github.com/ncnynl/xf-ros
  - 教程链接:https://www.ncnynl.com/archives/201702/1287.html
    - 主要是将`appid`换为自己的.
    - 并将`PACKAGE/lib/livmsc.so`换为自己的`SDK/lib/x64/livmsx.so`
# Darknet_ROS

+ ros下`物品识别`

+ Github地址：https://github.com/leggedrobotics/darknet_ros
  + 安装以后建议安装`NVidia`驱动以及`CUDA`和`CUDNN`，否则会出现`fps: 0.1`
  + 重新编译的时候需要删除`~/catkin_ws/devel`当中的`darknet_ros`文件夹，然后重新编译即可

# Zbar_ROS

+ `二维码`扫描封装

+ 知乎地址:https://zhuanlan.zhihu.com/p/40025902
+ zbar_opencv教程:https://blog.csdn.net/qq_33422184/article/details/77482015


# 附录

##  ROS WIKI-底盘控制：[velocity commands](http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/cmd_vel)

## ORK

+ ROS下的物品识别集成套件

+ 官方安装手册：http://wg-perception.github.io/object_recognition_core/index.html
  - [教程1](https://blog.csdn.net/u012057432/article/details/84068928)
  - [教程2](https://blog.csdn.net/weixin_42173928/article/details/86538387)
  - [教程3](https://blog.csdn.net/weixin_40799950/article/details/81911877)
  - 安装好以后检查并安装所需依赖(在catkin_ws下执行)
  ```sh
  rosdep check --from-paths /home/zzurobotics/catkin_ws/src/ork --ignore-src
  rosdep install --from-paths /home/zzurobotics/catkin_ws/src/ork --ignore-src
  ```
  - 将`ork_render`程序包中的`CMakeList.txt`当中的`mesa`方法修改为`GLUT`方法,[参考地址](https://github.com/JimmyDaSilva/ork_renderer/commit/4bdd53e3c418e7d02be0212ece04598619b4323a)
  - **OpenCV Error**: 由于kinectV2的分辨率所导致需要修改`/linemod/conf`文件夹下的`traning.ork`配置文件。以及`/linemod/src`文件夹下的`linemod_detect.cpp`文件.[参考地址](https://github.com/wg-perception/linemod/issues/28#issuecomment-200927751)
    + 修改完成以后别忘记 `catkin_make` 以及 `source` 
  ```shell
  rosrun topic_tools relay /kinect2/qhd/image_depth_rect /camera/depth_registered/image_raw
  rosrun topic_tools relay /kinect2/qhd/image_color_rect /camera/rgb/image_rect_color
  
  rosrun topic_tools relay /kinect2/qhd/camera_info /camera/rgb/camera_info
  
  rosrun topic_tools relay /kinect2/qhd/camera_info /camera/depth_registered/camera_info
  
  rosrun topic_tools relay /kinect2/qhd/points /camera/depth_registered/points
  
  rosrun tf static_transform_publisher 0 0 0 0 0 0 kinect2_ir_opticalrame camera_depth_optical_frame 40
  ```
## 安装CUDA(Compute Unified Device Architecture，统一计算架构[1])、CUDNN以及Nvidia驱动
 + 安装过程可参考马炜杰学长的博文：https://blog.csdn.net/DragonGirI/article/details/97614130
 + 名词解释：
   + cuDNN（CUDA Deep Neural Network library）：是NVIDIA打造的针对深度神经网络的加速库，是一个用于深层神经网络的GPU加速库。如果你要用GPU训练模型，cuDNN不是必须的，但是一般会采用这个加速库。
   + 知乎链接：[显卡、显卡驱动、cuda 之间的关系是什么？](https://www.zhihu.com/question/59184480)

