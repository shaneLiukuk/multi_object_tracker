# 20260420
  总结修改：

  新 Track 设计：

  1. 移除 matrix_/lst_flg_/cursor_ - 旧的扁平化环形缓冲区设计
  2. 新增每路传感器独立的 history：
  std::deque<FusedObject> sensor_history_svs_;     // max 30
  std::deque<FusedObject> sensor_history_bev_;     // max 30
  std::deque<FusedObject> sensor_history_radar_;   // max 30
  3. 新增 Sensor History 管理方法：
  void AddToSensorHistory(SensorType sensor_type, const FusedObject& obs);
  const std::deque<FusedObject>& GetSensorHistory(SensorType sensor_type) const;
  FusedObject GetPreviousSensorObject(SensorType sensor_type, int32_t offset) const;
  4. 使用 fused_object_ 替代 estimated_ - 更清晰地表示当前融合状态
  5. TrackerProcessor 适配新接口：
    - GetTrackObject() → GetFusedObject()
    - GetHistoryObject(idx) → GetPreviousSensorObject(sensor_type, offset)
    - SetCursorNext() → 移除（deque 自动管理）

● Bash(cp -f /home/nick/fusion_ws/src/od_fusion/include/od_fusion/lib/track.h /home/nick/od_fusion/include/od_fusion/lib/track.h && cp -f /home/nick/fusion_ws/src/od_f…)
  ⎿  Sync done

● 优化完成！

  主要改进：

  ┌────────────────────────────────────────────┬─────────────────────────────────────────────────────────────────┐
  │                   旧设计                   │                             新设计                              │
  ├────────────────────────────────────────────┼─────────────────────────────────────────────────────────────────┤
  │ matrix_ 扁平数组 (kTrackDepth×kTrackWidth) │ 每传感器独立 deque<FusedObject> (max 30)                        │
  ├────────────────────────────────────────────┼─────────────────────────────────────────────────────────────────┤
  │ cursor_ 环形索引管理                       │ deque 自动 push/pop                                             │
  ├────────────────────────────────────────────┼─────────────────────────────────────────────────────────────────┤
  │ GetTrackObject() / GetHistoryObject(idx)   │ GetFusedObject() / GetPreviousSensorObject(sensor_type, offset) │
  ├────────────────────────────────────────────┼─────────────────────────────────────────────────────────────────┤
  │ SetCursorNext() 手动移动光标               │ 无需手动管理                                                    │
  ├────────────────────────────────────────────┼─────────────────────────────────────────────────────────────────┤
  │ estimated_ 命名模糊                        │ fused_object_ 更清晰表达融合状态                                │
  └────────────────────────────────────────────┴─────────────────────────────────────────────────────────────────┘

  文件变更：
  - track.h: 新增 sensor_history_*、新增 AddToSensorHistory()、GetSensorHistory()、GetPreviousSensorObject()
  - track.cc: 完全重写 history 管理逻辑
  - tracker_processor.cc: 适配新 Track API
  - test_od_fusion.cc: 新增 SensorHistoryManagement 测试


  新增功能：在 UpdateWithoutSensorObject 中清理历史观测

  // track.h
  void PruneSensorHistory(SensorType sensor_type, uint64_t current_time);

  // track.cc - 新增方法
  void Track::PruneSensorHistory(SensorType sensor_type, uint64_t current_time) {
    // 获取该传感器最大不可见周期阈值
    float max_period = ...;  // svs/bev/radar 各自的最大不可见周期

    // 阈值为 max_period * 2 (如 max_period=0.5s, threshold=1.0s)
    double threshold = max_period * 2.0;

    // 移除超过阈值的旧观测
    while (!history->empty()) {
      double time_diff = (current_time - history->front().object.timestamp) / 1000.0;
      if (time_diff > threshold) {
        history->pop_front();
      } else {
        break;
      }
    }
  }

  // 在 UpdateWithoutSensorObject 中调用
  void Track::UpdateWithoutSensorObject(SensorType sensor_type, uint64_t meas_time) {
    ...
    SetPredicted(true);
    IncPredictionTime();

    // ✅ 清理该传感器历史中超过阈值的旧观测
    PruneSensorHistory(sensor_type, meas_time);
  }

  作用：
  - 当航迹未匹配到某传感器观测时，会清理该传感器历史中超过 max_period * 2 秒的旧观测
  - 例如：SVS 阈值 0.5s × 2 = 1.0s，超过 1 秒的 SVS 历史观测会被移除

  # 20260417
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