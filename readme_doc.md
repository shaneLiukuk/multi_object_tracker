# OD_FUSION 多传感器融合系统

## 1. 环境构建

### 1.1 系统要求

- Ubuntu 20.04 / 22.04
- ROS2 Humble
- Eigen3 (>= 3.4)
- CMake (>= 3.8)
- C++17 编译器

### 1.2 安装步骤

#### 1. 安装 ROS2 Humble

```bash
# 添加 ROS2 apt 源
sudo apt update
sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/rosdistro/ros.key
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/rosdistro/ros.key] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'

# 安装 ROS2
sudo apt update
sudo apt install ros-humble-desktop
```

#### 2. 安装 Eigen3

```bash
sudo apt install libeigen3-dev
```

#### 3. 创建工作空间

```bash
mkdir -p ~/fusion_ws/src
cd ~/fusion_ws/src
source /opt/ros/humble/setup.bash
colcon build
```

### 1.3 编译工程

```bash
# 进入工作空间
  cd /home/nick/fusion_ws

  # 1. 先编译 calmcar（生成 msg 头文件）
  source /opt/ros/humble/setup.bash
  colcon build --packages-select calmcar

  # 2. 再编译 od_fusion
  source install/setup.bash
  colcon build --packages-select od_fusion

  source /opt/ros/humble/setup.bash && colcon build --packages-select calmcar 2>&1
  source install/setup.bash && colcon build --packages-select od_fusion 2>&1

  # 3. 运行测试
  ./build/od_fusion/test_od_fusion

  # 4. 运行节点（需要实际传感器数据发布才能正常运行）
  ./build/od_fusion/od_fusion_node

  注意：运行时出现的 GLIBCXX_3.4.30 not found 错误是 Anaconda 与 ROS2 的 libstdc++ 版本冲突，解决方案：

  # 方案1：临时使用系统库
  export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH

  # 方案2：或启动时指定
  conda deactivate  # 退出 anaconda 环境
  ./build/od_fusion/od_fusion_node

```

## 2. 运行流程

### 2.1 启动融合节点

```bash
source /opt/ros/humble/setup.bash
/home/nick/od_fusion/build/od_fusion/od_fusion_node
```

### 2.2 节点配置

在 `fusion_node.cc` 中修改传感器使能配置：

```cpp
perception::fusion::OdFusionComponent::Config config;
config.enable_svs = true;      // 环视摄像头融合
config.enable_bev = false;      // BEV融合
config.enable_radar = false;    // 雷达融合
```

### 2.3 运行参数

- 融合周期：50ms
- 最大输出航迹数：64
- 航迹管理器深度：30
- 航迹管理器宽度：64

## 3. 目录结构

```
od_fusion/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # ROS2包描述
├── fusion_node.cc              # ROS2节点主程序
├── include/od_fusion/
│   ├── applications/
│   │   └── od_fusion_component.h   # 融合组件接口
│   ├── base/
│   │   └── obstacle_constant.h     # 基础数据结构和常量
│   └── lib/
│       ├── cv_kalman_filter.h     # 卡尔曼滤波器
│       ├── hungarian.h             # 匈牙利算法
│       ├── track.h                 # 航迹类
│       └── tracker_processor.h     # 航迹处理器
├── src/
│   ├── applications/
│   │   └── od_fusion_component.cc
│   └── lib/
│       ├── cv_kalman_filter.cc
│       ├── hungarian.cc
│       ├── track.cc
│       └── tracker_processor.cc
├── test/
│   └── test_od_fusion.cc           # 单元测试
└── launch/                         # 启动文件目录
```

## 4. 测试

### 4.1 运行单元测试

```bash
source /opt/ros/humble/setup.bash
/home/nick/od_fusion/build/od_fusion/test_od_fusion
```

### 4.2 测试用例说明

| 测试名称 | 说明 |
|---------|------|
| SingleFrameProcessing | 单帧处理测试 |
| MultiFrameProcessing | 多帧处理测试 |
| ContinuousTracking | 连续跟踪测试 |
| EmptyFrameProcessing | 空帧处理测试 |
| ObjectTypeConsistency | 目标类型一致性测试 |
| HighFrequencyProcessing | 高频处理测试 |

## 5. 常见问题

### 5.1 编译失败

- 检查 Eigen3 是否正确安装
- 确保使用 C++17 标准
- 验证 ROS2 环境变量已加载

### 5.2 运行失败

- 检查 ROS2 依赖是否完整
- 确认 `/opt/ros/humble/setup.bash` 已 source
