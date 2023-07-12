# auto_aim_fromRV
## 用法：
_ROS1 Noetic环境_
* ***step1:*** 把detector文件夹放到`workspace/src`里
* ***step2:*** 在workspace文件夹下`catkin_make`
* ***step3:*** ros打开realsense
* ***step4:*** `roslaunch detector detect.launch`开始识别

## 注意事项：
* 输入相机topic、输出相机topic、目标识别颜色可在`detector/launch/detect.launch`中修改
* 命令行输出识别到的装甲板个数、距离图像中心最近的装甲板位置、整套算法运行时间，并显示标记后的图像流
* 输出的图像流中心有个紫色的圈，若圈不在中心，则`camera_matrix`不准
* 相机参数、车到相机的tf未包装，在`detector/include/camera_info.hpp`中赋值，更改后要重新catkin_make
*  **输出的信息未包装**
* 默认：
  * `detect_color`为RED
  * 车到相机的tf为(0,0,0)：位移(x,y,z)
  * `camera_info`为小狗贴纸realsense的标定数据
* **opencv_test**包可用于拿视频或图片作为输入来测试算法，更改`opencv_test/src/opencv_test_node.cpp`中的代码就行
  * 使用方法和detector类似，`catkin_make`后
     * 测图片：`rosrun opencv_test opencv_test_node <image_path>`
     * 测视频：`rosrun opencv_test opencv_test_node`（视频路径在`opencv_test_node.cpp`里给）
