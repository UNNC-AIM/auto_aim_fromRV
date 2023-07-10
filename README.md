# auto_aim_fromRV
## 用法：
_ROS1 Noetic环境_
* ***step1:*** 把opencv_test放到workspace/src里
* ***step2:*** 在workspace文件夹下catkin_make
* ***step3:*** ros打开realsense
* ***step4:*** `rosrun opencv_test opencv_test_node`开始识别

## 现状：
* 输入相机topic: `/camera/color/image_raw`，命令行输出识别到的装甲板个数、距离图像中心最近的装甲板位置、整套算法运行时间，并显示标记后的图像流
*
  ***参数、输出的信息未包装***
* 所有参数都在.hpp文件内定义，要更改需要重新编译
*（例如：被识别的灯带颜色`detect_color`在detector_test.hpp顶部修改，
相机参数如`camera_matrix`和车到相机的tf在camera_info.hpp中改）*
* 默认：
  * `detect_color`为RED
  * 车到相机的tf为(0,0,0)：位移(x,y,z)
  * camera_info为小狗贴纸realsense的标定数据
