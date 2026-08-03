#!/bin/bash
sudo apt update
ROS_DISTRO=$(rosversion -d)
echo -e "\e[31m是否需要安装cartographer, 请输入y或n)\e[0m"
if [ "$input" == "y" ]; then
  echo "需要"
  if [ "$ROS_DISTRO" = "noetic" ]; then
    echo "for ROS Noetic"
    sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
  elif [ "$ROS_DISTRO" = "melodic" ]; then
    echo "for ROS Melodic"
    sudo apt-get install -y python-wstool python-rosdep ninja-build stow
  else  
    echo -e "\e[31mROS version is not Melodic or Noetic. Exit.\e[0m"
    exit 1
  fi
  sudo apt-get remove ros-$ROS_DISTRO-abseil-cpp
  cd ~
  git clone -b V0.0.1 https://gitee.com/reinovo/cartographer_ws.git
  cd cartographer_ws
  src/cartographer/scripts/install_abseil.sh
  catkin_make_isolated --install --use-ninja -j4
  echo -e "\e[31m查看编译信息, 输入按键确认cartographer是否编译成功(成功y,失败n)\e[0m"
  read input
  if [ "$input" == "y" ]; then
      echo "你输入了y,表示cartographer编译成功。"
  else
      echo "你输入了n或其他内容。你输入了y,表示cartographer编译失败。"
      exit
  fi
else
    echo "不需要"
fi

sudo apt install ros-$ROS_DISTRO-navigation* ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-teb-local-planner ros-$ROS_DISTRO-teleop-twist-keyboard libmodbus-dev sox ros-$ROS_DISTRO-dwa-local-planner -y
echo "开始安装YDLidar-SDK"
git clone -b V1.1.3 https://gitee.com/reinovo/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make&&sudo make install
cd ../..
rm YDLidar-SDK -rf
cp .reinovo ~ -rf
#sudo cp -p ./reinovo_bobac3.rules /etc/udev/rules.d/

echo "开始安装berxel-sdk"
cd ~
git clone -b dev_zy https://gitee.com/reinovo/berxel-sdk.git
cd berxel-sdk
CUR_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
echo "当前脚本目录: $CUR_DIR"
# 避免重复添加
if ! grep -q "export BERXEL_SDK_LIBRARY=" ~/.bashrc; then
    echo "export BERXEL_SDK_LIBRARY=$CUR_DIR" >> ~/.bashrc
fi
if ! grep -q "export LD_LIBRARY_PATH=.*$CUR_DIR/libs" ~/.bashrc; then
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$CUR_DIR/libs" >> ~/.bashrc
fi
echo "✅ 环境变量已永久写入 ~/.bashrc"
source ~/.bashrc

echo "开始安装reinovo-bobac3"
cd ~/bobac3_ws
catkin_make -j4

HOME_PATH=~
TARGET_STRING1="export REI_ROBOT"
TARGET_STRING2="export MAP_DIRECTORY"
TARGET_STRING3="export FACE_DIRECTORY"
TARGET_STRING4="bobac3_ws"
TARGET_STRING5="export LASER_TYPE"
sed -i "/$TARGET_STRING1/d" "$HOME_PATH/.bashrc"
sed -i "/$TARGET_STRING2/d" "$HOME_PATH/.bashrc"
sed -i "/$TARGET_STRING3/d" "$HOME_PATH/.bashrc"
sed -i "/$TARGET_STRING4/d" "$HOME_PATH/.bashrc"
sed -i "/$TARGET_STRING5/d" "$HOME_PATH/.bashrc"
echo "export REI_ROBOT=bobac3" >> ~/.bashrc
echo "export MAP_DIRECTORY=~/.reinovo/maps" >> ~/.bashrc
echo "export FACE_DIRECTORY=~/.reinovo/face_data" >> ~/.bashrc
echo "export LASER_TYPE="CSPC"" >> ~/.bashrc
source ~/.bashrc
echo "source ~/bobac3_ws/devel/setup.bash --extend" >> ~/.bashrc
source ~/.bashrc
