# multi_object_tracker

  新增 RvizDisplay 可视化类：

  1. 头文件 include/od_fusion/lib/rviz_display.h
    - PublishSvsObservations() - 发布SVS观测
    - PublishBevObservations() - 发布BEV观测
    - PublishRadarObservations() - 发布Radar观测
    - PublishFusionResults() - 发布融合结果
    - PublishAll() - 同时发布所有数据
  2. 实现 src/lib/rviz_display.cc
    - 3D盒子可视化（CUBE marker）
    - 尺寸处理：width/length/height为0时默认1米
    - 文本标签显示：ID、位置(X,Y)、速度(VX,VY)、总速度、类型、检测属性
    - 颜色区分：
        - SVS: 蓝色
      - BEV: 绿色
      - Radar: 红色
      - Fusion: 黄色
  3. 使用方法

  # 启动节点（发布到 /od_fusion/visualization）
  /home/nick/od_fusion/build/od_fusion/od_fusion_node

  # 在rviz2中订阅查看
  rviz2
  # 添加 MarkerArray 显示，topic选择 /od_fusion/visualization

  可视化效果：
  - 每帧发布3D方框表示目标
  - 方框上方显示文字标签包含位置、速度、类型信息
  - 不同传感器来源用不同颜色区分