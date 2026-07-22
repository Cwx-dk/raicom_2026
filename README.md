## 获取代码

### 下载
```bash
git clone --recurse-submodules https://gitee.com/reinovo/bobac3_ws.git
```

### 更新子模块

```bash
git submodule sync
git submodule update --init --recursive
```

### 编译

```bash
cd bobac3_ws/scripts
./install.sh
```

### 编译并安装

暂未进行修改，不建议使用。
```bash
$ catkin_make install -DCMAKE_INSTALL_PREFIX=/path/to/install
```

## 仿真

### 环境配置

```bash
cd bobac3_ws/src/bobac3_description/scripts
./install.sh
```

### 启动仿真环境

本工作空间配备了bobac3机器人仿真环境。
功能包：
- bobac3_description：机器人模型描述以及仿真环境，仿真运行gazebo.launch
- bobac3_slam：地图构建，仿真运行bobac3_slam_sim.launch
- bobac3_navigation：导航，仿真运行demo_nav_2d.launch


