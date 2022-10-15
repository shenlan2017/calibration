# Cyber Bag to ROS bag

## 1 测试环境

```
gcc >= 5, recommend gcc >= 9.4
cmake >= 3.15.0
Eigen >= 3.2.7
boost >= 1.58.0
OpenCV 3.x.x
yaml-cpp 0.7.0
ros >= kinetic
```

## 2 使用方法

1. 修改`config/config.txt`中的相关参数

   + `generate_rosbag_name`：为新创建的 rosbag 包名，默认放在 repo 的目录下
   + `dag_folder_path`：Apollo导出的数据路径，详见`apollo_shenlan`仓库中的`dump_message_all.py`文件；
   + `xxx_topic`：在ros包中发布的话题，目前仅支持部分数据类型；此外，轮式里程计数据为了方便存储，以`QuaternionStamped` 类型进行发布，先后顺序为：`rr,rl,fr,fl`；

2. 放在ros工作目录下进行编译，并运行；

   ```bash
   catkin_make -j4
   source devel/setup.bash
   roslaunch apollo2ros start.launch
   ```

## 3 目前支持数据类型及之后安排

- [x] GNSS数据：WGS84坐标系下LLA及LLA方差，对应`best_pose`文件夹；
- [x] IMU数据：对应`imu`文件夹；
- [x] 轮式里程计数据：记录四轮的线速度信息，对应`chassis`文件夹；
- [x] 激光数据：对应`lidar_pcd`文件夹；
- [x] rtk数据：采用里程计数据进行导出，由组合惯导直接导出，对应`rtk_pose`文件夹；
- [ ] 相机图像数据
- [ ] gnss信号状态数据
- [ ] 毫米波雷达数据
- [ ] 定位数据



