### topic_demo

> 功能描述：两个node，一个发布GPS消息(格式自定义，包括坐标和工作状态)，另一个接受并处理该信息(计算到原点的距离)

#### 具体步骤

+ 创建package

```sh
cd ~/catkin_ws/src
catkin_create_pkg topic_demo roscpp rospy std_msgs
```

+ 定义msg

```sh
cd topic_demo
mkdir msg
cd msg
vi gps.msg
```

+ gps.msg定义如下

  ```c++
  float32 x  
  floar32 y  
  string state  
  ```

  + 编译以后会在`～/catkin_ws/devel/include/topic_demo`路径下生成`gps.h`文件
  + 在自己的程序当中下需要包含`#include<topic_demo/gps.h>`头文件，并通过`topic_demo::gps msg`使用此数据类型

+ talker.cpp

```c
  #include <ros/ros.h>   
  #include <topic_demo/gps.h>  //自定义msg产生的头文件

  int main(int argc, char **argv){
    ros::init(argc, argv, "talker");  //解析参数，第三个参数为本节点名
    ros::NodeHandle nh;    //实例化句柄，实例化node
    topic_demo::gps msg;  //自定义gps消息对象并初始化

    //我们可以把该对象当成结构体来使用。
    msg.x=1.0;
    msg.y=1.0;
    msg.data="working";

    ros::Publisher pub = nh.advertise<topic_demo::gps>("gps_info", 1); //创建publisher，往"gps_info"话题上发布消息
    ros::Rate loop_rate(1.0);   //定义发布的频率，1HZ
    while(ros::ok())   //循环发布msg.只要ros没有关闭就就一直进行循环
    {
      //处理msg
      msg.x*=1.03;
      pub.publish(msg);//发布msg
      loop_rate.sleep();//根据前面的定义的loop_rate,设置1s的暂停
    }
    return 0;
  }
```

+ listener.cpp

```c++
 #include<ros/ros.h>
 #include<topic_demo/gps.h>
 void gpsCallback(const topic_demo::gps::ConstPtr& msg){
   //我们的topic_demo::gps这个类下定义了ConstPtr这个常量指针类型
   std_msgs::FLoat32 distance; //Float32.msg当中进行了定义。一种ros下自带的msg数据类型。
   //msg在此是指针，访问指针的成员变量的时候使用->.另外distance当中的data变量存放的是真正的数据
   distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
   ROS_INFO("Distance is %f.\nState is %s",distance.data,msg->state.c_str());
}

 int main(int argc,char** argv){
   ros::init(argc,argv,"listener");
   ros::NodeHandle nh;
   ros::Subscriber sub=n.subscribe("gps_info",1,gpsCallback);//创建subscriber
   //ros::spin 函数 反复调用当前可触发的回调函数，阻塞。
   //ros::spinOnce 函数 如果有可触发的触发函数，则调用回调函数，对当前的message进行处理。否则，往下运行
   ros::sipn();//反复查看当前的消息队列当中是否有东西。如果有的话则会一个一个的清空该队列。并调用回调函数对message进行处理.
   return 0;
  }
```

+ 修改CMakeList和package.xml

  + CMakelist.txt

  ```sh
  cmake_minimum_required(VERSION 2.8.3) #cmake版本
  project(topic_demo) #项目名称
  #指定依赖
  find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation   #该依赖项可以自动帮我们生成自定义的msg文件
  )
  
  add_message_files(FILES gps.msg)  #指定我们生成msg时候具体从那个文件里面生成
  #catkin在cmake之上新增的命令，指定从哪个消息文件生成
  
  generate_messages(DEPENDENCIES std_msgs) #将msg文件生成为对应的头文件
  #catkin新增的命令，用于生成消息
  #DEPENDENCIES后面指定生成msg需要依赖其他什么消息，由于gps.msg用到了flaot32这种ROS标准消息，因此需要再把std_msgs作为依赖
  
  catkin_package(CATKIN_DEPENDS roscpp rospy stdmsgs message_runtime)
  #用于配置ROS和package配置文件和Cmake文件
  
  include_directories(include ${catkin_INCLUDE_DIRS}) #头文件路径
  
  add_executable(talker src/talker.cpp) #生成可执行文件talker
  add_dependencies(talker topic_demo_generate_messages_cpp)
  #表明在编译talker前，必须先编译完成自定义消息
  #必须添加add_dependencies，否则找不到自定义的msg产生的头文件
  target_link_libraries(talker ${catkin_LIBRARIES}) #链接
  
  add_executable(listener src/listener.cpp ) #生成可执行文件listener
  add_dependencies(listener topic_demo_generate_messages_cpp)
  target_link_libraries(listener ${catkin_LIBRARIES})#链接
  ```

  + package.xml

  ```xml
  <!--声明依赖-->
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</run_depend>
  ```

### 
