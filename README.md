# auto_aim_fromRV
## 用法：
* _ROS1 Noetic环境_ 
### step1:
**新建workspace：**   
$`mkdir -p ~/aim_ws/src`  
$`cd ~/aim_ws`  
$`catkin_make`  
  
**添加路径：**  
$`gedit ~/.bashrc`  
在文件末尾加上`source <workspace的路径>/devel/setup.bash`  
$`source ~/.bashrc`  
### step2:
**从github上clone:**  
在aim_ws/src中$`git clone https://github.com/UNNC-AIM/auto_aim_fromRV.git`  
  
* *其中的opencv_test包可以删去，是detector的草稿版*
* *tracker包是目标状态跟踪器，能跑但效果很不理想，可能是因为没有TF转换（realsense D455的rgb相机坐标系和相机整体坐标系间还需要位姿转换一下，可以通过rqt_tf_tree或者rviz中的tf来看，此包现在没有这一层转换）*
* ***detector包中的`set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)`这一句需要注释掉，是我的电脑上的opencv的路径，如果之后catkin_make因为找不到opencv报错的话，把这一句中的路径换成自己的opencv所在的路径***
> （2023.10.22）对于x86处理器而言一般不需要注释？因为安装OpenCV的时候会把所有.cmake文件放在者个目录下（似乎），但是对于OrangePi而言就需要改为`/usr/lib/aarch64-linux-gnu/cmake/opencv4`，猜测一切arm架构处理器上都需要这样操作。此外，注释掉是错误行为，因为100%会报错 --wxq
  

### step3:
$`cd ~/aim_ws`  
$`catkin_make`  
  
**如果catkin_make失败而且不知道怎么办，或者有其他问题的话，可以找我一起看看(scysw3@nottingham.ac.uk)*

### detector用法：
* ros打开realsense
* 用不了realsense的话，直接用电脑前置摄像头或者别的网络摄像头也行：
  * 用ros打开你的摄像头
  * 将detect.launch中的`input_topic`参数改为你自己摄像头的视频流的topic
  * 更改`detector/include/camera_info.hpp`中的相机参数为你自己的相机参数（更改后要重新catkin_make），相机参数可以通过查看topic`/camera_info`（也许为其他名字）来看，如果不准的话需要标定，参考http://wiki.ros.org/camera_calibration
* $`roslaunch detector detect.launch` 开始识别装甲板
* 可以在rviz中添加`/camera/color/image_raw`的topic来查看标记后的影像
* 装甲板的位姿、类型等信息在topic`/detector/armors`中，可用$`rostopic echo /detector/armors`来查看

#### 注意事项：
* 输入相机topic、输出相机topic、目标识别颜色、车到相机的tf可在`detector/launch/detect.launch`中修改
* 输出的图像流中心有个紫色的圈，若圈不在中心，则`camera_matrix`不准，得去`detector/include/camera_info.hpp`中修改
* 相机参数未包装，在`detector/include/camera_info.hpp`中赋值，更改后要重新catkin_make
* 默认：
  * `detect_color`为RED
  * 车到相机的tf为(0,0,0)：位移(x,y,z)
  * `camera_info`为小狗贴纸realsense的标定数据

### tracker用法：
* 在已启动detector的前提下，$`rosrun tracker tracker_node`
* 打开rviz，查看topic`/tracker/marker`，就能查看整车的运动状态，但肯定是不理想的，你有空的话研究一下源码然后改进一下，理想状态是显示的车怎么运动，marker也怎么运动。
* 现状(可以建出模但是不能跟踪运动)：![image](/shouldbe.png)

  
***auto_aim_msgs包里是自定义的消息类型，被detector和tracker调用***

**opencv_test**包可用于拿视频或图片作为输入来测试算法，更改`opencv_test/src/opencv_test_node.cpp`中的代码就行
  * 使用方法和detector类似，`catkin_make`后
     * 测图片：`rosrun opencv_test opencv_test_node <image_path>`
     * 测视频：`rosrun opencv_test opencv_test_node`（视频路径在`opencv_test_node.cpp`里给）
