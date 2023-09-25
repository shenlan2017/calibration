# shenlan_calibration安装说明

[toc]

## 0.安装依赖

```sh
#cmake升级
https://blog.csdn.net/Boys_Wu/article/details/104940575
wget https://github.com/Kitware/CMake/releases/download/v3.21.4/cmake-3.21.4-linux-x86_64.tar.gz
tar -xzvf cmake-3.21.4-linux-x86_64.tar.gz
sudo mv cmake-3.21.4-linux-x86_64 /opt/cmake-3.21.4  
sudo ln -sf /opt/cmake-3.21.4/bin/* /usr/bin/
cmake --version
sudo gedit ~/.bashrc
export PATH=$PATH:/opt/cmake-3.21.4/bin
source ~/.bashrc

#g++升级
https://blog.csdn.net/tytyvyibijk/article/details/123074333
add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-11 g++-11
ls /usr/bin/gcc*
ls /usr/bin/g++*
sudo update-alternatives --remove-all gcc
sudo update-alternatives --remove-all g++
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 1
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 10
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 1
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 10
gcc --version
g++ --version

#clang升级
sudo apt-get install clang

#安装依赖
sudo apt install libboost-dev libopencv-dev libeigen3-dev libpcl-dev libceres-dev libyaml-cpp-dev
```

## 1.camera_intrinsics_ws

### 1.1.cam_collect

编译：

```sh
cd ~/shenlan/calibration/camera_intrinsics_ws/cam_collect_ros_ws

catkin_make
```

运行：

```sh
cd ~/shenlan/calibration/camera_intrinsics_ws/cam_collect_ros_ws   

source devel/setup.bash

roslaunch usb_cam usb_cam.launch

cd ~/shenlan/calibration/camera_intrinsics_ws/cam_collect_ros_ws   

source devel/setup.bash

roslaunch cam_collect cam_collect.launch
```

### 1.2.calibration_kit

编译：

```sh
sudo apt install libboost-dev libopencv-dev libeigen3-dev libpcl-dev libceres-dev libyaml-cpp-dev

cd ~/shenlan/calibration/camera_intrinsics_ws/calibration_kit_docker/workspace/calibration_kit

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

cmake --build build --parallel 4
```

运行：

```sh
cd ~/shenlan/calibration/camera_intrinsics_ws/calibration_kit_docker/workspace/calibration_kit/build

./calibration_kit
```

## 2.camera2lidar_ws

### 2.1.Pangolin

编译：

```sh
cd ~/shenlan/calibration/3rdparty/Pangolin

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

cmake --build build --parallel 4
```

### 2.2.camera2lidar

编译：

```sh
cd ~/shenlan/calibration/camera2lidar_ws

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

cmake --build build --parallel 4
```

运行：

```sh
cd ~/shenlan/calibration/camera2lidar_ws/bin

./run_lidar2camera \
../test/1.jpeg \
../test/1.pcd \
../test/front_6mm_intrinsics.yaml \
../test/front_6mm_extrinsics.yaml
```

## 3.lidar2ins_non_ros_ws

### 3.1.ceres

编译：

```sh
cd ~/shenlan/calibration/3rdparty/ceres-solver

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

cmake --build build --parallel 4
```

### 3.2.pose_align

编译：

```sh
cd ~/shenlan/calibration/lidar2ins_non_ros_ws/pose_align

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

cmake --build build --parallel 4
```

运行：

```sh
cd ~/shenlan/calibration/lidar2ins_non_ros_ws/pose_align/build

./pose_align /home/neousys/apollo/data/bag/calib_lidar2ins/parsed_data/00000/pcd
```

## 4.lidar2ins_ros_ws

编译：

```sh
sudo apt-get install libglm-dev libglfw3-dev

sudo apt-get install ros-melodic-geodesy ros-melodic-pcl-ros ros-melodic-nmea-msgs ros-melodic-libg2o

cd ~/shenlan/calibration/lidar2ins_ros_ws

catkin_make -j4
```

运行：

```sh
roscore

cd ~/shenlan/calibration/lidar2ins_ros_ws

source devel/setup.bash

rosrun interactive_slam odometry2graph
```



