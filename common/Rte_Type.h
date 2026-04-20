/*
 * File: Rte_Type.h
 *
 * Code generated for Simulink model 'Fusion'.
 *
 * Model version                  : 1.3661
 * Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
 * C/C++ source code generated on : Fri Apr 17 15:37:46 2026
 */

#ifndef RTW_HEADER_Rte_Type_h_
#define RTW_HEADER_Rte_Type_h_
#include "rtwtypes.h"

typedef struct {
  int32_T x[6];
  int32_T y[6];
  uint8_T type[6];
  uint64_T time_stamp;
} Fus_UpaObjectInfo;

typedef struct {
  real32_T x;
  real32_T y;
  uint8_T class_type;
  uint8_T edge;
  uint16_T id;
  real32_T vehicle_x;
  real32_T vehicle_y;
  real32_T vehicle_heading;
  uint8_T fs_point_source;
} CcPoint2f;

typedef struct {
  real_T x;
  real_T y;
  real_T width;
  real_T height;
} Rect;

typedef struct {
  uint32_T crc;
  uint16_T data_len;
  uint8_T rc;
  uint8_T reserved[3];
} DataCheck;

typedef struct {
  uint16_T secondshi;
  uint32_T seconds;
  uint32_T nanoseconds;
} TimestampHeader;

typedef int16_T sint16;
typedef struct {
  uint8_T uss_echo_available[12];
  uint8_T uss_echo_model[12];
  uint64_T first_echo_timestamp_us[12];
  uint16_T first_echo_distance[12];
  uint8_T first_echo_prob[12];
  uint16_T first_echo_width[12];
  uint16_T first_echo_amplitudel[12];
  uint16_T second_echo_distance[12];
  uint8_T second_echo_prob[12];
  uint16_T second_echo_width[12];
  uint16_T second_echo_amplitudel[12];
  uint16_T third_echo_distance[12];
  uint8_T third_echo_prob[12];
  uint16_T third_echo_width[12];
  uint16_T third_echo_amplitudel[12];
} USSEchoInfo;

typedef struct {
  real32_T x;
  real32_T y;
  uint8_T class_type;
  uint8_T edge;
  uint8_T point_type;
  uint16_T id;
  real32_T vehicle_x;
  real32_T vehicle_y;
  real32_T vehicle_heading;
  uint8_T motion_status;
  uint8_T point_status2;
  uint8_T point_status1;
  real32_T point_info1;
  real32_T point_info2;
} FusPoint2f;

typedef struct {
  uint16_T distance;
  uint8_T type;
  uint16_T width;
  uint8_T peak;
  uint8_T counter;
  uint64_T time_stamp;
  uint8_T reserve;
} Fus_ApaDistT;

typedef struct {
  Fus_UpaObjectInfo upa_object[3];
} Fus_UpaObjsT;

typedef struct {
  uint16_T distance[12];
  uint64_T time_stamp;
} Fus_UpaDistT;

typedef struct {
  /* 0-无效
     其他-有效 */
  real32_T distance_x;

  /* 0-无效
     其他-有效 */
  real32_T distance_y;

  /* 0-未跟踪
     其他-跟踪 */
  uint16_T tracking_id;

  /* 0-unknow
     1-car
     2-bus
     3-truck
     4-person
     5-bicycle(motorcycle)
     6-cyclist
     7-tricycle
     8-parking lock open
     9-parking lock close
     10-pillar 橙色或红色柱子/可升降柱子/站立的限位杆
     11-waterhouse 水马/水桶型路障
     12-fire hydrant 消防栓
     13-trash can 垃圾桶
     14-cones 锥桶
     15-balance car 平衡车
     16-scooter 滑板车
     17-Trailer 平板车
     18-ground warning sign 地面施工警示牌
     19-wheel block 轮挡
     20-baby stroller 婴儿车
     21-shopping cart 购物推车
     22-Speherical obstable 球形障碍物/类球形墩子
     23-limiting stopper 车位里的限位块
     24-limiting rod 车位里躺放的限位杆
     25-Stop rod 闸机上悬空的停止杆或落地的栅栏
     26-地下停车场立柱
   */
  uint8_T class_id;

  /* 0-unknow
     1-left_behind
     2-left_front
     3-right_behind
     4-right_front
     5-right
     6-left
     7-front
     8-behind */
  uint8_T angle_view;

  /* —— */
  uint8_T confidence;

  /* 0-无效
     其他-有效 */
  real32_T width;

  /* 0-无效
     其他-有效 */
  real32_T height;

  /* 0-无效
     其他-有效 */
  real32_T length;

  /* 0-无效
     大于-1及小于1返回内表示目标角度为0
     其他-有效 */
  real32_T yaw;

  /* 0-无效
     大于-0.01 小于0.01表示为0
     其他-有效 */
  real32_T relative_velocity_x;

  /* 0-无效
     大于-0.01 小于0.01表示为0
     其他-有效 */
  real32_T relative_velocity_y;

  /* 0-无效
     大于-0.01 小于0.01表示为0
     其他-有效 */
  real32_T relative_acceleration_x;

  /* 0-无效
     大于-0.01 小于0.01表示为0
     其他-有效 */
  real32_T relative_acceleration_y;

  /* 0-unknow
     1-Moving
     2-Oncoming
     3-Stationary
     4-StationaryCandidate
     5-CrossingStationary
     6-CrossingMoving
     7-Stopped */
  uint8_T motion_status;

  /* 0-8（从低到高）每bit位分别表示width、height、length、yaw、relative_velocity_x、relative_velocity_y、relative_acceleration_x、relative_acceleration_y、motion_status是否有效 */
  uint16_T valid_status;

  /* 0-Unknow
     1-Front Camera
     2-Surround Camera
     3-Front Radar
     4-Left Front Radar
     5-Left Behind Radar
     6-Right Front Radar
     7-RIGHT Behind Radar
     8-Uss
     9-Fusion */
  uint8_T target_source;
} Fusion_ObjectInfo;

typedef struct {
  /* 唯一id */
  uint8_T id;

  /* 障碍物类型 */
  uint8_T type;

  /* 横向相对距离 */
  real32_T rel_distance_x;

  /* 纵向相对距离 */
  real32_T rel_distance_y;

  /* 横向相对速度 */
  real32_T rel_velocity_x;

  /* 纵向相对速度 */
  real32_T rel_velocity_y;

  /* 横向相对加速度 */
  real32_T rel_acceleration_x;

  /* 纵向相对加速度 */
  real32_T rel_acceleration_y;

  /* 相对航向角 */
  real32_T heading;

  /* 跟踪次数 */
  uint8_T tracking_time;
  uint8_T detect_state;

  /* 目标相对于本车的运动方向 */
  uint8_T motion_orientation;
  uint8_T motion_state;

  /* 匹配的观测值id */
  uint8_T match_ids;

  /* 类型置信度 */
  real32_T type_confidence;

  /* 绝对航向角 */
  real32_T yaw;

  /* 横向绝对距离 */
  real32_T abs_distance_x;

  /* 纵向绝对距离 */
  real32_T abs_distance_y;

  /* 横向绝对速度 */
  real32_T abs_velocity_x;

  /* 纵向绝对速度 */
  real32_T abs_velocity_y;

  /* 横向绝对加速度 */
  real32_T abs_acceleration_x;

  /* 纵向绝对加速度 */
  real32_T abs_acceleration_y;

  /* 障碍物来源 */
  uint8_T source;

  /* 障碍物长度 */
  real32_T length;

  /* 障碍物高度 */
  real32_T height;

  /* 障碍物宽度 */
  real32_T width;

  /* 时间戳 */
  uint64_T timestamp;

  /* 车辆目标的刹车灯是否点亮 */
  uint8_T brake_light;

  /* 当前车辆左转灯是否闪烁 */
  uint8_T turn_indicatoleft;

  /* 当前车辆右转灯是否闪烁 */
  uint8_T turn_indicatoright;

  /* 目标在哪个车道 */
  uint8_T obj_lane_assignment;

  /* 目标是否有效 */
  uint8_T obj_lane_assignment_v;

  /* 预留 */
  uint8_T track_od_info1;

  /* 预留 */
  uint8_T track_od_info2;

  /* 预留 */
  real32_T track_od_info3;

  /* 预留 */
  real32_T track_od_info4;
} Fus_TrackedObjectInfo;

typedef struct {
  /* 时间戳 */
  int64_T timestamp_raw;

  /* 帧号 */
  int64_T frame_index;

  /* 相机位置
     0=CENTER
     1=CENTER_NEAR(中间的120°）
     2=FRONT_LEFT
     3=FRONT_RIGHT
     4=REAR_CENTER
     5=REAR_LEFT
     6=REAR_RIGHT
     7=FRONT_CENTER_FAR */
  uint8_T camera_position;

  /* 目标id */
  uint8_T id;

  /* kObjectBackground = 0
     kObjectCar = 1
     kObjectBus = 2
     kObjectTruck = 3
     kObjectPerson = 4
     kObjectCyclist = 5 */
  uint8_T class_a;

  /* 目标类别名称
     "Background"
     "Car"
     "Bus"
     "Truck"
     "Pedestrian"
     "Cyclist" */
  uint8_T class_name;

  /* 目标子类别 */
  uint8_T subclass;

  /* 目标子类别名称 */
  uint8_T subclass_name;

  /* 置信度 */
  real32_T confidence;

  /* 目标长度 */
  real32_T length;

  /* 目标长度标准差 */
  real32_T length_std;

  /* 目标宽度 */
  real32_T width;

  /* 目标宽度标准差 */
  real32_T width_std;

  /* 目标高度 */
  real32_T height;

  /* 目标高度标准差 */
  real32_T height_std;

  /* 目标存在帧数 */
  uint8_T age_count;

  /* 目标存在的时间 */
  real32_T age_seconds;

  /* 目标观测视角
     VS_NOT_VISIBLE = 0
     VS_FRONT = 1
     VS_REAR = 2 */
  uint8_T visibility_side;

  /* 目标航向角 */
  real32_T heading;

  /* 目标航向角标准差 */
  real32_T heading_std;

  /* 目标碰撞时间 */
  real32_T inverse_ttc;

  /* 碰撞时间标准差 */
  real32_T inverse_ttc_std;

  /* 本车纵轴与目标前部/后部的左侧之间的角度 */
  real32_T angle_left;

  /* 本车纵轴与目标前部/后部的右侧侧之间的角度 */
  real32_T angle_right;

  /* 本车纵轴与目标远可见边缘之间的角度 */
  real32_T angle_side;

  /* 目标的角速度 */
  real32_T angle_rate;

  /* 目标是否从图像上方越界
     0=FALSE
     1=TRUE */
  uint8_T top_out_of_image;

  /* 目标是否从图像下方越界
     0=FALSE
     1=TRUE */
  uint8_T bottom_out_of_image;

  /* 目标是否从图像左边越界
     0=FALSE
     1=TRUE */
  uint8_T left_out_of_image;

  /* 目标是否从图像右边越界
     0=FALSE
     1=TRUE */
  uint8_T right_out_of_image;

  /* 刹车灯是否点亮
     0=FALSE
     1=TRUE */
  uint8_T brake_light;

  /* 左转灯是否闪烁
     0=FALSE
     1=TRUE */
  uint8_T turn_indicator_left;

  /* 右转灯是否闪烁
     0=FALSE
     1=TRUE */
  uint8_T turn_indicator_right;

  /* 目标状态度量
     bit0: enum MeasuringStatus0 {
     MS_OLD = 0
     MS_NEW = 1 }
     bit1: enum MeasuringStatus1 {
     MS_PREDICTED = 0
     MS_MEASURED = 1 }
     bit2: enum MeasuringStatus2 {
     MS_INVALID = 0
     MS_VALID = 1 }
     bit3: message MeasuringStatus {
     MeasuringStatus0 measuring_status_0 = 1
     MeasuringStatus1 measuring_status_1 = 2
     MeasuringStatus2 measuring_status_2 = 3} */
  uint8_T measuring_status;

  /* 目标的运动方向
     MO_INVALID = 0
     MO_DRIFTING_RIGHT = 1
     MO_CROSSING_RIGHT = 3
     MO_OC_DRIFTING_RIGHT = 5
     MO_ONCOMING = 6
     MO_OC_DRIFTING_LEFT = 7
     MO_CROSSING_LEFT = 9
     MO_DRIFTING_LEFT = 11
     MO_PRECEEDING = 12
     MO_UNKNOWN = 13 */
  uint8_T motion_orientation;

  /* 目标的运动类型
     MC_UNFILLED = 0
     MC_UNDEFINED = 1
     MC_PASSING = 2
     MC_PASSING_IN = 3
     MC_PASSING_OUT = 4
     MC_CLOSE_CUT_IN = 5
     MC_MOVING_IN = 6
     MC_MOVING_OUT = 7
     MC_CROSSING = 8
     MC_LTAP = 9
     MC_RTAP = 10
     MC_MOVING = 11
     MC_PRECEEDING = 12
     MC_ONCOMING = 13 */
  uint8_T motion_category;

  /* 目标运动状态
     0=Invalid
     1=Unknown
     2=Moving
     3=Stationary
     4=Stopped
     5=Moving_Slowly
     MOTION_STATUS_UNKNOWN = 0
     MOTION_STATUS_PARKED = 1
     MOTION_STATUS_STOPPED = 2
     MOTION_STATUS_MOVING = 3
     MOTION_STATUS_ONCOMING = 4
     MOTION_STATUS_CROSSING = 5
     MOTION_STATUS_STATIONARY=6 */
  uint8_T motion_status;

  /* 目标的切入切出状态
     CUT_IN_CUT_OUT_UNKNOWN = 0
     CUT_IN_CUT_OUT_HOST_CUT_IN_LEFT = 1
     CUT_IN_CUT_OUT_HOST_CUT_IN_RIGHT = 2
     CUT_IN_CUT_OUT_HOST_CUT_OUT_LEFT = 3
     CUT_IN_CUT_OUT_HOST_CUT_OUT_RIGHT = 4
     CUT_IN_CUT_OUT_NO_CUT_IN_OUT = 5 */
  uint8_T cutin_cutout;

  /* 横向距离 */
  real32_T lat_distance;

  /* 横向距离标准差 */
  real32_T lat_distance_std;

  /* 纵向距离 */
  real32_T long_distance;

  /* 纵向距离标准差 */
  real32_T long_distance_std;

  /* 横向相对速度 */
  real32_T relative_lat_velocity;

  /* 横向相对速度标准差 */
  real32_T relative_lat_velocity_std;

  /* 纵向相对速度 */
  real32_T relative_long_velocity;

  /* 纵向相对速度标准差 */
  real32_T relative_long_velocity_std;

  /* 横向绝对速度 */
  real32_T abs_lat_velocity;

  /* 横向绝对速度标准差 */
  real32_T abs_lat_velocity_std;

  /* 纵向绝对速度 */
  real32_T abs_long_velocity;

  /* 纵向绝对速度标准差 */
  real32_T abs_long_velocity_std;

  /* 横向相对加速度 */
  real32_T relative_lat_acc;

  /* 横向相对加速度标准差 */
  real32_T relative_lat_acc_std;

  /* 纵向相对加速度 */
  real32_T relative_long_acc;

  /* 纵向相对加速度标准差 */
  real32_T relative_long_acc_std;

  /* 横向绝对加速度 */
  real32_T abs_lat_acc;

  /* 横向绝对加速度标准差 */
  real32_T abs_lat_acc_std;

  /* 纵向绝对加速度 */
  real32_T abs_long_acc;

  /* 纵向绝对加速度标准差 */
  real32_T abs_long_acc_std;

  /* 绝对速度 */
  real32_T abs_speed;

  /* 绝对速度标准差 */
  real32_T abs_speed_std;

  /* 绝对加速度 */
  real32_T abs_acceleration;

  /* 绝对加速度标准差 */
  real32_T abs_acceleration_std;

  /* 0=UNKNOWN
     1=LEFT_LEFT
     2=LEFT
     3=HOST
     4=RIGHT
     5=RIGHT_RIGHT */
  uint8_T obj_lane_assignment;

  /* 0: 不是
     1：是 */
  uint8_T is_bev_object;
} Fus_PerceptionBEVObjectInfo;

typedef struct {
  /* 时间戳 */
  uint64_T timestamp;

  /* 跟踪ID */
  uint32_T track_id;

  /* 出现的帧数 */
  uint32_T age;

  /* 存在概率 */
  real32_T exist_probability;

  /* 车道线质量
     0=VERY_LOW
     1=LOW
     2=PREDICTED
     3=RELIABLE
     4=STABLEe
     5=HIGH */
  uint8_T quality;

  /* 预测原因
     0=(BIT_0)LOSS
     1=(BIT_1)HEADWAY
     2=(BIT_2)OCCLUDED
     3=(BIT_3)DIVERGING
     4=(BIT_4)GR_SHADOW
     5=(BIT_5)MERGE */
  uint64_T prediction_source;

  /* 预测类型
     0=(BIT_0)OTHER_SIDE
     1=(BIT_1)PARTIAL */
  uint64_T prediction_type;

  /* 车道线颜色
     0=UNDECIDED
     1=WHITE
     2=YELLOW
     3=BLUE_GREEN
     4=ORANGE_RED */
  uint8_T color;

  /* 颜色置信度 */
  real32_T color_confidence;

  /* 车道线类型
     0=UNDECIDED
     1=SOLID
     2=DASHED
     3=DLM
     4=BOTTS
     5=DECELERATION
     6=HOV_LANE */
  uint8_T lanemark_type;

  /* 车道线类型置信度 */
  real32_T lanemark_type_confidence;

  /* 车道线双线类型
     0=NOT_DLM
     1=SOLID_DASHED
     2=DASHED_SOLID
     3=SOLID_SOLID
     4=DASHED_DASHED
     5=UNDECIDED */
  uint8_T dlm_type;

  /* 车道线减速类型
     0=NO_DECEL
     1=SOLID
     2=DASHED
     3=RESERVED */
  uint8_T decel_type;

  /* 车道线的位置
     0=UNKNOWN
     1=LEFT
     2=RIGHT */
  uint8_T side;

  /* 标志是否交叉
     0=FALSE
     1=TRUE */
  uint8_T crossing;

  /* 车道线宽度 */
  real32_T marker_width;

  /* 车道线宽度标准差 */
  real32_T marker_width_std;

  /* 虚线时虚线各部分的平均长度 */
  real32_T dash_average_length;

  /* 虚线部分之间的平均间距 */
  real32_T dash_average_gap;

  /* 标记选择车道线方程信号段
     0=FALSE
     1=TRUE */
  uint8_T is_multi_clothoid;

  /* 第一段曲线三次方程c0参数 */
  real32_T first_line_c0;

  /* 第一段曲线段三次方程c1参数 */
  real32_T first_line_c1;

  /* 第一段曲线段三次方程c2参数 */
  real32_T first_line_c2;

  /* 第一段曲线段三次方程c3参数 */
  real32_T first_line_c3;

  /* 第一段车道标志纵向视野范围的起点 */
  real32_T first_view_range_start;

  /* 第一段车道标志纵向视野范围的终点 */
  real32_T first_view_range_end;

  /* 第一段车道标志纵向测量视野范围的终点 */
  real32_T first_measured_view_range_end;

  /* 第二段曲线三次方程c0参数 */
  real32_T second_line_c0;

  /* 第二段曲线段三次方程c1参数 */
  real32_T second_line_c1;

  /* 第二段曲线段三次方程c2参数 */
  real32_T second_line_c2;

  /* 第二段曲线段三次方程c3参数 */
  real32_T second_line_c3;

  /* 第二段车道标志纵向视野范围的起点 */
  real32_T second_view_range_start;

  /* 第二段车道标志纵向视野范围的终点 */
  real32_T second_view_range_end;

  /* 第二段车道标志纵向测量视野范围的终点 */
  real32_T second_measured_view_range_end;

  /* 相机位置
     0=CENTER
     1=CENTER_NEAR
     2=FRONT_LEFT
     3=FRONT_RIGHT
     4=REAR_CENTER
     5=REAR_LEFT
     6=REAR_RIGHT
     7=FRONT_CENTER_FAR */
  uint8_T camera_position;

  /* 标记车道线是否可用（内部）
     0=FALSE
     1=TRUE */
  uint8_T is_valid;

  /* 图像点列（内部）(reserve) */
  uint32_T image_points;
} Fus_LaneHost;

typedef struct {
  /* 跟踪ID */
  uint32_T track_id;

  /* 出现的帧数 */
  uint32_T age;

  /* 存在概率 */
  real32_T exist_probability;

  /* 车道线质量
     0=VERY_LOW
     1=LOW
     2=PREDICTED
     3=RELIABLE
     4=STABLEe
     5=HIGH */
  uint8_T quality;

  /* 预测原因
     0=NONE
     1=FULL */
  uint64_T prediction_type;

  /* 预测类型
     0=NONE
     1=LOSS */
  uint64_T prediction_source;

  /* 车道线颜色
     0=UNDECIDED
     1=WHITE
     2=YELLOW
     3=BLUE_GREEN
     4=ORANGE_RED */
  uint8_T color;

  /* 颜色置信度 */
  real32_T color_confidence;

  /* 车道线类型
     0=UNDECIDED
     1=SOLID
     2=DASHED
     3=DLM
     4=BOTTS
     5=DECELERATION
     6=HOV_LANE */
  uint8_T lanemark_type;

  /* 车道线类型置信度 */
  real32_T lanemark_type_confidence;

  /* 车道线双线类型
     0=NOT_DLM
     1=SOLID_DASHED
     2=DASHED_SOLID
     3=SOLID_SOLID
     4=DASHED_DASHED
     5=UNDECIDED */
  uint8_T dlm_type;

  /* 车道线的位置
     0=NONE
     1=LEFT__LEFT_LANEMARK
     2=LEFT__RIGHT_LANEMARK
     3=RIGHT__LEFT_LANEMARK
     4=RIGHT__RIGHT_LANEMARK */
  uint8_T role;

  /* 车道线宽度 */
  real32_T marker_width;

  /* 车道线宽度标准差 */
  real32_T marker_width_std;

  /* 曲线段三次方程中c3参数 */
  real32_T line_c3;

  /* 曲线段三次方程中c2参数 */
  real32_T line_c2;

  /* 曲线段三次方程中c1参数 */
  real32_T line_c1;

  /* 曲线段三次方程中c0参数 */
  real32_T line_c0;

  /* 车道标志纵向视野范围的起点 */
  real32_T view_range_start;

  /* 车道标志纵向视野范围的终点 */
  real32_T view_range_end;

  /* 车道标志纵向测量视野范围的终点 */
  real32_T measured_view_range_end;

  /* 时间戳 */
  uint64_T timestamp;

  /* 相机位置
     0=CENTER
     1=CENTER_NEAR
     2=FRONT_LEFT
     3=FRONT_RIGHT
     4=REAR_CENTER
     5=REAR_LEFT
     6=REAR_RIGHT
     7=FRONT_CENTER_FAR */
  uint8_T camera_position;

  /* 标记车道线是否可用（内部）
     0=FALSE
     1=TRUE */
  uint8_T is_valid;

  /* 图像点列（内部）(reserve) */
  uint32_T image_points;
} Fus_LaneAdjacent;

typedef struct {
  /* 跟踪ID */
  uint32_T id;

  /* 出现的帧数 */
  uint32_T age;

  /* 存在概率 */
  real32_T exist_probability;

  /* 类型
     0=UNDECIDED
     1=ROAD_EDGE
     2=ELEVATED_STRUCTURE
     3=CURB
     4=CONES_POLES
     5=PARKING_CARS */
  uint8_T type;

  /* 预测类型
     1=OCCLUDED
     2=OTHER_SIDE
     3=OVERRIDE
     4=DIST_BASED_EXTRAPOLATION
     5=HEADWAY_ORIENTED */
  uint8_T prediction_type;

  /* 线的位置
     0=UNFILLED
     1=LEFT
     2=RIGHT */
  uint8_T side;

  /* 线的顺序
     0=UNFILLED
     1=FIRST
     2=SECOND */
  uint8_T index;

  /* 高度 */
  real32_T height;

  /* 高度标准差 */
  real32_T height_std;

  /* 曲线段三次方程中c3参数 */
  real32_T line_c3;

  /* 曲线段三次方程中c2参数 */
  real32_T line_c2;

  /* 曲线段三次方程中c1参数 */
  real32_T line_c1;

  /* 曲线段三次方程中c0参数 */
  real32_T line_c0;

  /* 车道标志纵向视野范围的起点 */
  real32_T view_range_start;

  /* 车道标志纵向视野范围的终点 */
  real32_T view_range_end;

  /* 车道标志纵向测量视野范围的终点 */
  real32_T measured_view_range_end;

  /* 时间戳 */
  uint64_T timestamp;

  /* 相机位置 */
  uint8_T camera_position;

  /* 标记车道线是否可用（内部） */
  uint8_T is_valid;

  /* 图像点列（内部）（reserve） */
  uint32_T image_points;
} Fus_RoadEdge;

typedef struct {
  /* 目标数量 */
  uint8_T barricade_count;

  /* 时间戳 */
  uint64_T timestamp;

  /* 帧号 */
  uint64_T frame_index;

  /* 0=CENTER
     1=CENTER_NEAR
     2=FRONT_LEFT
     3=FRONT_RIGHT
     4=REAR_CENTER
     5=REAR_LEFT
     6=REAR_RIGHT
     7=FRONT_CENTER_FAR */
  uint8_T camera_position;

  /* 目标id */
  uint8_T id;

  /* 目标类别
     kObjectWarning = 6
     kObjectProhibit = 7
     kObjectIndicate = 8
     kObjectFingerpost = 9
     kObjectTravel = 10
     kObjectSupplement = 11,  kObjectSignalLamp = 12
     kObjectEtcLight = 13
     kObjectOther = 14
     kObjectPerceptionFrame.Barricade = 15
     kObjectSignalSpecial = 16 */
  uint8_T class_obj;

  /* (string)reserve,当前目标的类别名称 */
  real_T class_name;

  /* reserve当前目标的子类别 */
  uint8_T subclass;

  /* (string)reserve当前目标的子类别名称 */
  real_T subclass_name;

  /* 置信度 */
  real32_T confidence;

  /* 目标宽度 */
  real32_T width;

  /* 目标宽度标准差 */
  real32_T width_std;

  /* 目标长度 */
  real32_T length;

  /* 目标长度标准差 */
  real32_T length_std;

  /* 目标航向角 */
  real32_T heading;

  /* 目标航向角标准差 */
  real32_T heading_std;

  /* 目标高度 */
  real32_T height;

  /* 目标高度标准差 */
  real32_T height_std;

  /* 目标存在帧数 */
  uint8_T age_count;

  /* 目标距地面高度 */
  real32_T height_from_ground;

  /* 目标距地面高度标准差 */
  real32_T height_from_ground_std;

  /* 横向距离，以目标的几何中心为参考点 */
  real32_T lat_distance;

  /* 横向距离标准差 */
  real32_T lat_distance_std;

  /* 纵向距离，以目标的几何中心为参考点 */
  real32_T long_distance;

  /* long_distance_std+D21:R26D27DD21:Q26纵向距离标准差 */
  real32_T long_distance_std;

  /* 0: 不是
     1：是 */
  uint8_T detect_source_is_bev_object;
} Fus_Barriercade;

typedef struct {
  uint64_T time_stamp;
  uint32_T frame_index;
  real32_T p0_x;
  real32_T p0_y;
  real32_T p1_x;
  real32_T p1_y;
  real32_T p2_x;
  real32_T p2_y;
  real32_T p3_x;
  real32_T p3_y;
  real32_T width;
  real32_T depth;
  uint8_T direction;
  uint8_T slot_type;
  uint8_T slot_status;
  uint8_T slot_detection_type;
  uint8_T slot_occupied;
  uint8_T quality;
  real32_T angle;
  real32_T lane_width;
  uint8_T tracking_id;
  uint8_T serial_number[16];
} ParkingSlot;

typedef struct {
  uint16_T point_num;
  uint8_T closed_contour;
  CcPoint2f vehicle_points[400];
  uint8_T color_type;
  uint8_T class_type;
} FreespaceInfo;

typedef struct {
  real32_T distance_x;
  real32_T distance_y;
  uint16_T tracking_id;
  uint8_T class_id;
  uint8_T angle_view;
  uint8_T confidence;
  real32_T width;
  real32_T height;
  real32_T length;
  real32_T yaw;
  real32_T relative_velocity_x;
  real32_T relative_velocity_y;
  real32_T relative_acceleration_x;
  real32_T relative_acceleration_y;
  uint8_T motion_status;
  uint16_T valid_status;
  uint8_T target_source;
  real32_T dimension_variance_x;
  real32_T dimension_variance_y;
  real32_T distance_variance_x;
  real32_T distance_variance_y;
  real32_T relative_velocity_variance_x;
  real32_T relative_velocity_variance_y;
  real32_T relative_acc_variance_x;
  real32_T relative_acc_variance_y;
  real32_T yaw_variance;
  real32_T object_life_time;
  real32_T object_pre_life_time;
  uint16_T sensor_confirm;
} ObjectInfo;

typedef struct {
  real32_T steering_angle;
  uint8_T steering_enable;
  real32_T steering_column_torque;
  uint8_T driver_override;
  uint8_T steering_control_available;
  real32_T steering_angle_speed;
  uint8_T steering_flt_status;
  uint64_T can_time;
  uint64_T system_time;
} SteeringInfo;

typedef struct {
  uint8_T braking_status;
  real32_T brake_pedal_output;
  real32_T master_cylinder_pressure;
  uint8_T brake_control_available;
  uint8_T brake_error;
  uint8_T brake_enable;
  uint8_T driver_override;
  uint8_T external_brake_priority;
  uint8_T abs_enable;
  uint8_T abs_active;
  uint8_T scs_resp_status;
  real32_T ibsrespacctoqval;
  uint8_T autoholdsyssts;
  uint8_T peb_dcl_req_resp;
  uint64_T can_time;
  uint64_T system_time;
} BrakeInfo;

typedef struct {
  real32_T wheel_speed_rear_right;
  real32_T wheel_speed_rear_left;
  real32_T wheel_speed_front_right;
  real32_T wheel_speed_front_left;
  uint8_T wheel_speed_rear_right_dir;
  uint8_T wheel_speed_rear_left_dir;
  uint8_T wheel_speed_front_right_dir;
  uint8_T wheel_speed_front_left_dir;
  int16_T wheel_position_rear_right;
  int16_T wheel_position_rear_left;
  int16_T wheel_position_front_right;
  int16_T wheel_position_front_left;
  real32_T vehicle_speed;
  real32_T vehspdavgdrvn;
  real32_T vehspdavgnondrvn;
  uint8_T vehicle_stand_still;
  real32_T longitude_acce;
  uint8_T longitude_acce_valid;
  real32_T lateral_acce;
  uint8_T lateral_acce_valid;
  real32_T roll_rate;
  real32_T yaw_rate;
  uint64_T can_time;
  uint64_T system_time;
} VehicleSpeedInfo;

typedef struct {
  real32_T throttle_pedal_output;
  real32_T torque_output_fr;
  real32_T torque_output_rr;
  uint8_T throttle_enable;
  uint8_T driver_override;
  uint8_T throttle_error;
  uint8_T throttle_control_available;
  uint8_T apa_reverse_req_resp;
  uint64_T can_time;
  uint64_T system_time;
} ThrottleInfo;

typedef struct {
  uint8_T gear_status;
  uint8_T gear_enable;
  uint8_T driver_override;
  uint8_T scu_flt_status;
  uint64_T can_time;
  uint64_T system_time;
} AutoGearInfo;

typedef struct {
  uint8_T epb_status;
  uint8_T epb_switch_status;
  uint8_T epb_availability_status;
  uint8_T epb_apa_availability_status;
  uint8_T driver_override;
  uint8_T epb_enable;
  uint8_T epb_flt_status;
  uint64_T can_time;
  uint64_T system_time;
} EPBInfo;

typedef struct {
  uint8_T turn_light_status;
  uint8_T turn_light_switch_status;
  uint8_T hazard_mod_status;
  uint8_T brake_light_status;
  uint8_T high_beam;
  uint8_T low_beam;
  uint8_T wiper;
  uint8_T door_driver;
  uint8_T door_driver_switch_status;
  uint8_T door_passenger;
  uint8_T door_passenger_switch_status;
  uint8_T door_left_rear;
  uint8_T door_right_rear;
  uint8_T hood;
  uint8_T trunk;
  uint8_T passenger_detect;
  uint8_T driver_detect;
  uint8_T driver_seat_belt;
  uint8_T passenger_seat_belt;
  real32_T temperature;
  uint8_T temperature_mask;
  uint8_T vehicle_lock_status;
  uint8_T reserve_1;
  uint8_T reserve_2;
  uint8_T reserve_3;
  uint8_T reserve_4;
  uint8_T reserve_5;
  uint8_T reserve_6;
  uint16_T reserve_7;
  uint16_T reserve_8;
  uint16_T reserve_9;
  uint16_T reserve_10;
  real32_T reserve_11;
  real32_T reserve_12;
  real32_T reserve_13;
  real32_T reserve_14;
  uint64_T can_time;
  uint64_T system_time;
} BodyInfo;

typedef struct {
  uint16_T front_left_pressure;
  uint16_T front_right_pressure;
  uint16_T rear_left_pressure;
  uint16_T rear_right_pressure;
  uint8_T front_left_pressure_alarm;
  uint8_T front_right_pressure_alarm;
  uint8_T rear_left_pressure_alarm;
  uint8_T rear_right_pressure_alarm;
  uint8_T tire_pressure_monitor_sys_flt;
  uint64_T can_time;
  uint64_T system_time;
} TirePressureInfo;

typedef struct {
  uint8_T power_mode_ctrl_avlbl;
  uint8_T vehicle_mode;
  uint8_T usage_mode;
  uint8_T system_power_mode;
  uint8_T mode_of_power_mode;
  uint8_T power_mode_control_response;
  uint8_T electric_powertrain_ready_status;
  uint8_T vehicle_driving_mode;
  uint8_T reserve_1;
  uint8_T reserve_2;
  uint8_T reserve_3;
  uint8_T reserve_4;
  uint64_T can_time;
  uint64_T system_time;
} PowerInfo;

typedef struct {
  uint8_T airbag_deployed_status;
  uint8_T airbag_deployed_inversion;
  uint8_T airbag_sys_flt_status;
  uint8_T reserve_1;
  uint8_T reserve_2;
  uint8_T reserve_3;
  uint8_T reserve_4;
  uint16_T reserve_5;
  uint16_T reserve_6;
  uint16_T reserve_7;
  uint16_T reserve_8;
  real32_T reserve_9;
  real32_T reserve_10;
  uint64_T can_time;
  uint64_T system_time;
} AirbagInfo;

typedef struct {
  uint8_T charger_plug_connection_status;
  uint8_T reserve_1;
  uint8_T reserve_2;
  uint8_T reserve_3;
  uint8_T reserve_4;
  uint16_T reserve_5;
  uint16_T reserve_6;
  uint16_T reserve_7;
  uint16_T reserve_8;
  real32_T reserve_9;
  real32_T reserve_10;
  uint64_T can_time;
  uint64_T system_time;
} ChargeInfo;

typedef struct {
  uint8_T reserve_1;
  uint8_T reserve_2;
  uint8_T reserve_3;
  uint8_T reserve_4;
  uint8_T reserve_5;
  uint8_T reserve_6;
  uint8_T reserve_7;
  uint8_T reserve_8;
  uint8_T reserve_9;
  uint8_T reserve_10;
  uint16_T reserve_11;
  uint16_T reserve_12;
  uint16_T reserve_13;
  uint16_T reserve_14;
  real32_T reserve_15;
  real32_T reserve_16;
  real32_T reserve_17;
  real32_T reserve_18;
  real32_T reserve_19;
  real32_T reserve_20;
  real32_T reserve_21;
  real32_T reserve_22;
  uint64_T can_time;
  uint64_T system_time;
} ReserveInfo;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_can;
  real32_T x;
  real32_T y;
  real32_T z;
  real32_T heading;
  real32_T pitch;
  real32_T roll;
} GlobalPoseEstimation;

typedef struct {
  int64_T timestamp_raw;
  int64_T frame_index;
  uint8_T camera_position;
  uint8_T id;
  uint8_T class_a;
  uint8_T class_name;
  uint8_T subclass;
  uint8_T subclass_name;
  real32_T confidence;
  Rect bbox;
  real32_T length;
  real32_T length_std;
  real32_T width;
  real32_T width_std;
  real32_T height;
  real32_T height_std;
  uint8_T age_count;
  real32_T age_seconds;
  uint8_T visibility_side;
  real32_T heading;
  real32_T heading_std;
  real32_T inverse_ttc;
  real32_T inverse_ttc_std;
  real32_T angle_left;
  real32_T angle_left_std;
  real32_T angle_right;
  real32_T angle_right_std;
  real32_T angle_side;
  real32_T angle_side_std;
  real32_T angle_rate;
  uint8_T top_out_of_image;
  uint8_T bottom_out_of_image;
  uint8_T left_out_of_image;
  uint8_T right_out_of_image;
  uint8_T brake_light;
  uint8_T turn_indicator_left;
  uint8_T turn_indicator_right;
  uint8_T measuring_status;
  uint8_T motion_orientation;
  uint8_T motion_category;
  uint8_T motion_status;
  uint8_T cutin_cutout;
  real32_T lat_distance;
  real32_T lat_distance_std;
  real32_T long_distance;
  real32_T long_distance_std;
  real32_T relative_lat_velocity;
  real32_T relative_lat_velocity_std;
  real32_T relative_long_velocity;
  real32_T relative_long_velocity_std;
  real32_T abs_lat_velocity;
  real32_T abs_lat_velocity_std;
  real32_T abs_long_velocity;
  real32_T abs_long_velocity_std;
  real32_T relative_lat_acc;
  real32_T relative_lat_acc_std;
  real32_T relative_long_acc;
  real32_T relative_long_acc_std;
  real32_T abs_lat_acc;
  real32_T abs_lat_acc_std;
  real32_T abs_long_acc;
  real32_T abs_long_acc_std;
  real32_T abs_speed;
  real32_T abs_speed_std;
  real32_T abs_acceleration;
  real32_T abs_acceleration_std;
  uint8_T obj_lane_assignment;
  uint8_T is_bev_object;
} PerceptionBEVObjectInfo;

typedef struct {
  uint64_T timestamp;
  uint32_T track_id;
  uint32_T age;
  real32_T exist_probability;
  uint8_T quality;
  uint64_T prediction_source;
  uint64_T prediction_type;
  uint8_T color;
  real32_T color_confidence;
  uint8_T lanemark_type;
  real32_T lanemark_type_confidence;
  uint8_T dlm_type;
  uint8_T decel_type;
  uint8_T side;
  uint8_T crossing;
  real32_T marker_width;
  real32_T marker_width_std;
  real32_T dash_average_length;
  real32_T dash_average_gap;
  uint8_T is_multi_clothoid;
  real32_T first_line_c0;
  real32_T first_line_c1;
  real32_T first_line_c2;
  real32_T first_line_c3;
  real32_T first_view_range_start;
  real32_T first_view_range_end;
  real32_T first_measured_view_range_end;
  real32_T second_line_c0;
  real32_T second_line_c1;
  real32_T second_line_c2;
  real32_T second_line_c3;
  real32_T second_view_range_start;
  real32_T second_view_range_end;
  real32_T second_measured_view_range_end;
  uint8_T camera_position;
  uint8_T is_valid;
  uint32_T image_points;
} LaneHost;

typedef struct {
  uint32_T track_id;
  uint32_T age;
  real32_T exist_probability;
  uint8_T quality;
  uint64_T prediction_type;
  uint64_T prediction_source;
  uint8_T color;
  real32_T color_confidence;
  uint8_T lanemark_type;
  real32_T lanemark_type_confidence;
  uint8_T dlm_type;
  uint8_T role;
  real32_T marker_width;
  real32_T marker_width_std;
  real32_T line_c3;
  real32_T line_c2;
  real32_T line_c1;
  real32_T line_c0;
  real32_T view_range_start;
  real32_T view_range_end;
  real32_T measured_view_range_end;
  uint64_T timestamp;
  uint8_T camera_position;
  uint8_T is_valid;
  uint32_T image_points;
} LaneAdjacent;

typedef struct {
  uint32_T id;
  uint32_T age;
  real32_T exist_probability;
  uint8_T type;
  uint8_T prediction_type;
  uint8_T side;
  uint8_T index;
  real32_T height;
  real32_T height_std;
  real32_T line_c3;
  real32_T line_c2;
  real32_T line_c1;
  real32_T line_c0;
  real32_T view_range_start;
  real32_T view_range_end;
  real32_T measured_view_range_end;
  uint64_T timestamp;
  uint8_T camera_position;
  uint8_T is_valid;
  uint32_T image_points;
} RoadEdge;

typedef struct {
  uint8_T exit_merge_available;
  uint8_T lap_is_highway_exit_left_bo;
  uint8_T lap_is_highway_exit_right_bo;
  uint8_T lap_is_highway_merge_left_bo;
  uint8_T lap_is_highway_merge_right_bo;
  uint8_T diversion_sign;
} MergeExitIndicator;

typedef struct {
  uint8_T throttle_enable;
  uint8_T throttle_ignore;
  uint8_T throttle_system_mode;
  uint16_T throttle_command;
  uint8_T target_acce;
} ThrottleCommand;

typedef struct {
  uint8_T steer_enable;
  uint8_T steer_system_mode;
  uint8_T steer_clear;
  real32_T steering_command;
  real32_T steering_speed_command;
} SteerCommand;

typedef struct {
  uint8_T brake_enable;
  uint8_T brake_standstill_req;
  uint8_T brake_driver_off_req;
  uint8_T brake_system_mode;
  uint8_T brake_system_flt_mode;
  uint8_T brake_acc_req_status;
  real32_T brake_command_acc_req_val;
  real32_T brake_command_toq_req_val;
  uint8_T dec_to_stop_flag;
  uint8_T brake_control_mode;
  uint8_T peb_dcl_req_sts;
  real32_T peb_dcl_req_toq_val;
} BrakeCommand;

typedef struct {
  uint8_T gear_enable;
  uint8_T gear_ignore;
  uint8_T gear_clear;
  uint8_T gear_command;
} GearCommand;

typedef struct {
  uint8_T epb_enable;
  uint8_T epb_command;
  uint8_T reserve_1;
  uint8_T reserve_2;
  uint32_T reserve_3;
  uint32_T reserve_4;
  real32_T reserve_5;
  real32_T reserve_6;
} EPBCommand;

typedef struct {
  uint8_T barricade_count;
  uint64_T timestamp;
  uint64_T frame_index;
  uint8_T camera_position;
  uint8_T id;
  uint8_T class_obj;
  real_T class_name;
  uint8_T subclass;
  real_T subclass_name;
  real32_T confidence;
  real32_T width;
  real32_T width_std;
  real32_T length;
  real32_T length_std;
  real32_T heading;
  real32_T heading_std;
  real32_T height;
  real32_T height_std;
  uint8_T age_count;
  real32_T height_from_ground;
  real32_T height_from_ground_std;
  real32_T lat_distance;
  real32_T lat_distance_std;
  real32_T long_distance;
  real32_T long_distance_std;
  uint8_T detect_source_is_bev_object;
} Barriercade;

typedef struct {
  DataCheck data_check;
  TimestampHeader time_header;
} MessageHeader;

typedef struct {
  uint16_T uss_obj_id;
  int16_T uss_obj_p1_x;
  int16_T uss_obj_p1_y;
  int16_T uss_obj_p2_x;
  int16_T uss_obj_p2_y;
  uint8_T uss_obj_prob;
  uint8_T uss_obj_height_conf;
  uint8_T uss_obj_height_status;
  uint8_T uss_obj_type;
  uint64_T uss_obj_timesatmp_us;
  uint8_T uss_obj_Attri;
} UssObjData;

typedef struct {
  uint8_T id;
  uint8_T type;
  real32_T width;
  real32_T length;
  real32_T height;
  real32_T rcs;
  uint8_T moving_sts;
  uint8_T measured_st;
  real32_T rel_vel_x;
  real32_T rel_vel_y;
  real32_T abs_vel_x_var;
  real32_T abs_vel_y_var;
  real32_T abs_vel_x;
  real32_T abs_vel_y;
  real32_T abs_accel_x_var;
  real32_T abs_accel_y_var;
  real32_T abs_accel_x;
  real32_T abs_accel_y;
  real32_T dist_x_var;
  real32_T dist_y_var;
  real32_T dis_x;
  real32_T dis_y;
  real32_T global_dis_x;
  real32_T global_dis_y;
  real32_T orin_angle;
  uint8_T exist_prob;
  uint8_T obj_confidence;
  uint8_T dyn_prob;
  uint8_T target_source;
  real32_T obstacle_prob;
  uint8_T obj_valid;
  real32_T ret_0;
  uint8_T ret_1;
  uint8_T ret_2;
} LRRObjectInfo;

typedef struct {
  uint8_T uss_system_status;
  uint8_T uss_status[12];
  uint8_T uss_de_enable[12];
  sint16 uss_de_temperature[12];
  USSEchoInfo uss_de_echo;
} UssDeStruct;

typedef struct {
  USSEchoInfo uss_left_echo;
  USSEchoInfo uss_right_echo;
} UssCeStruct;

typedef struct {
  uint8_T upc_parkingslot_id;
  uint64_T upc_parkingslot_timestamp;
  uint8_T upc_parkingslot_type;
  uint16_T upc_parkingslot_length;
  uint16_T upc_parkingslot_depth;
  uint8_T upc_parkingslotpoint1_objecttype;
  uint16_T upc_parkingslotpoint1_position_x;
  uint16_T upc_parkingslotpoint1_position_y;
  uint16_T upc_parkingslotpoint1_position_angle;
  uint8_T upc_parkingslotpoint2_objecttype;
  uint16_T upc_parkingslotpoint2_position_x;
  uint16_T upc_parkingslotpoint2_position_y;
  uint16_T upc_parkingslotpoint2_position_angle;
  uint8_T upc_parkingslot_objecttype;
  uint16_T upc_parkingslotdepthpoint1_position_x;
  uint16_T upc_parkingslotdepthpoint1_position_y;
  uint16_T upc_parkingslotdepthpoint2_position_x;
  uint16_T upc_parkingslotdepthpoint2_position_y;
} UssPSData;

typedef struct {
  uint8_T id;
  uint8_T type;
  real32_T rel_distance_x;
  real32_T rel_distance_y;
  real32_T rel_velocity_x;
  real32_T rel_velocity_y;
  real32_T rel_acceleration_x;
  real32_T rel_acceleration_y;
  real32_T heading;
  uint8_T tracking_time;
  uint8_T detect_state;
  uint8_T motion_orientation;
  uint8_T motion_state;
  uint8_T match_ids;
  real32_T type_confidence;
  real32_T yaw;
  real32_T abs_distance_x;
  real32_T abs_distance_y;
  real32_T abs_velocity_x;
  real32_T abs_velocity_y;
  real32_T abs_acceleration_x;
  real32_T abs_acceleration_y;
  uint8_T source;
  real32_T length;
  real32_T height;
  real32_T width;
  uint64_T timestamp;
  uint8_T brake_light;
  uint8_T turn_indicatoleft;
  uint8_T turn_indicatoright;
  uint8_T obj_lane_assignment;
  uint8_T obj_lane_assignment_v;
  uint8_T track_od_info1;
  uint8_T track_od_info2;
  real32_T track_od_info3;
  real32_T track_od_info4;
} TrackedObjectInfo;

typedef struct {
  uint64_T time_stamp;
  uint32_T frame_index;
  real32_T p0_x;
  real32_T p0_y;
  real32_T p1_x;
  real32_T p1_y;
  real32_T p2_x;
  real32_T p2_y;
  real32_T p3_x;
  real32_T p3_y;
  real32_T width;
  real32_T depth;
  uint8_T direction;
  uint8_T slot_type;
  uint8_T slot_status;
  uint8_T slot_detection_type;
  uint8_T slot_occupied;
  uint8_T quality;
  real32_T angle;
  real32_T lane_width;
  uint8_T tracking_id;
  real32_T vehicle_passage_width;
  uint8_T parking_direction;
  uint8_T parking_type;
  uint8_T slot_attributes[16];
  uint8_T slot_info1;
  uint8_T slot_info2;
  real32_T chock_x;
  real32_T chock_y;
} FusionParkingSlotToPlanAndControl;

typedef struct {
  uint16_T point_num;
  uint8_T closed_contour;
  FusPoint2f vehicle_points[400];
  uint8_T color_type;
  uint8_T class_type;
} FusFreespaceInfo;

typedef struct {
  /* 时间戳 */
  uint64_T time_stamp;

  /* fls,frs,rrs,rls */
  Fus_ApaDistT apa_dist_info[4];

  /* 0:front
     1:rear */
  Fus_UpaObjsT upa_objs[2];

  /* 0:front
     1:rear */
  Fus_UpaDistT upa_dist[2];

  /* 无效：0  有效：其他 */
  uint16_T min_distance[12];
} Fus_UssInfo;

typedef struct {
  uint64_T time_stamp;
  uint8_T counter;
  uint8_T processing_status;
} Fus_WorkingStatus;

typedef struct {
  /* 时间戳 */
  uint64_T time_stamp;

  /* 时间戳 */
  uint64_T time_stamp_ins;

  /* x方向加速度 */
  real32_T acce_x;

  /* y方向加速度 */
  real32_T acce_y;

  /* z方向加速度 */
  real32_T acce_z;

  /* x方向角速率 */
  real32_T gyro_x;

  /* y方向角速率 */
  real32_T gyro_y;

  /* z方向角速率 */
  real32_T gyro_z;

  /* Heading 偏航角（0～359.99） */
  real32_T heading;

  /* 俯仰角（-90～90） */
  real32_T pitch;

  /* 横滚角（-180～180） */
  real32_T roll;

  /* IMU的温度 */
  real32_T imu_temperature;
} Fusion_IMUInfo;

typedef struct {
  /* 0x00:Disabled
     0x01:Enabled */
  real32_T x;

  /* 转向系统的工作模式 */
  real32_T y;

  /* 1 = 清除接管标志
     0 = 不改变 */
  real32_T z;
} StVector3f;

typedef struct {
  /* message发送时间戳 */
  uint64_T time_stamp;

  /* 目标被检测到对应的rawdata时间戳 */
  uint64_T time_stamp_raw;

  /* 障碍物检测对应的帧号（仅视觉赋值） */
  uint32_T frame_index;

  /* 障碍物数量 */
  uint8_T object_cnt;

  /* NULL-表示空，没有目标
     其他表示为有目标 */
  Fusion_ObjectInfo object_set[64];
} Fusion_ObjectSet;

typedef struct {
  /* message发送时间戳 */
  uint64_T time_stamp;

  /* 每个周期加1 */
  uint8_T counter;

  /* 障碍物数量 */
  uint8_T object_cnt;

  /* NULL-表示空，没有目标 其他表示为有目标 */
  Fus_TrackedObjectInfo tracked_object_set[64];
} Fus_TrackedObjects;

typedef struct {
  /* msg发送时间戳 */
  uint64_T timestamp;
  uint8_T vd_count;
  uint8_T vru_count;
  uint8_T num_of_objects;
  uint8_T cipv_id;
  uint8_T vd_niv_left;
  uint8_T vd_niv_right;

  /* cipv目标丢失原因
     0=NO_LOSS
     1=LOST_TARGET_FOV_OUT
     2=LOST_TARGET_FOV_IN */
  uint8_T cipv_lost;

  /* acc工作状况
     0=FREE_SPACE
     1=SPACE_NOT_FREE
     2=FREE_SPACE_UNKNOWN */
  uint8_T allow_acc;
  Fus_PerceptionBEVObjectInfo cam_objects[40];
} Fus_PerceptionBEVObject;

typedef struct {
  /* 当前信号同步帧号 */
  uint64_T sync_id;

  /* 此协议支持的最大Host个数 */
  uint8_T lane_host_count;

  /* Lane.LaneHost信号组 */
  Fus_LaneHost lane_host[4];

  /* 此协议支持的最大adjacent个数 */
  uint8_T lane_adjacent_count;

  /* Lane.LaneAdjacent信号组 */
  Fus_LaneAdjacent lane_adjacent[4];

  /* 此协议支持的最大road_edge个数 */
  uint8_T road_edge_count;

  /* Lane.LaneRoadedge信号组 */
  Fus_RoadEdge lane_road[4];

  /* 用于预测或其他驾驶提示/场景提示的主车道宽度的多帧估计 */
  uint8_T estimated_width;
} Fus_RoadLaneSet;

typedef struct {
  /* msg发送时间戳 */
  uint64_T timestamp;

  /* 系统预留，表示barriercade的实际有效数量 */
  uint8_T obj_cnt;

  /* 系统预留位 */
  uint64_T sys_reserve0;

  /* 静态od数量，最多23个 */
  Fus_Barriercade barriercade[30];
} Fus_BarriercadeSet;

typedef struct {
  uint64_T time_stamp;
  uint8_T enable_parking_slot_detection;
  uint8_T enable_object_detection;
  uint8_T enable_freespace_detection;
  uint8_T enable_uss;
  uint8_T enable_radar;
  uint8_T enable_lidar;
  uint8_T system_command;
  uint8_T system_reset;
} PerceptionCommand;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_slot;
  uint32_T frame_index;
  uint8_T slot_cnt;
  ParkingSlot slot_set[26];
} ParkingSlotSet;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_camera;
  uint32_T frame_index;
  uint8_T info_cnt;
  FreespaceInfo freespace_set;
} FreespaceSet;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_raw;
  uint32_T frame_index;
  uint8_T object_cnt;
  ObjectInfo object_set[64];
} ObjectSet;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_can;
  SteeringInfo steering;
  BrakeInfo brake;
  VehicleSpeedInfo vehicle_speed;
  ThrottleInfo throttle;
  AutoGearInfo gear;
  EPBInfo epb;
} CarInfoH;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_can;
  BodyInfo body_info;
  TirePressureInfo tire_pressure;
  PowerInfo poewe_info;
  AirbagInfo airbag_info;
  ChargeInfo charge_info;
  ReserveInfo reserve_info;
} CarInfoL;

typedef struct {
  uint8_T counter;
  GlobalPoseEstimation global_pose[10];
} GlobalPoseBuffer;

typedef struct {
  uint64_T timestamp;
  uint8_T vd_count;
  uint8_T vru_count;
  uint8_T num_of_objects;
  uint8_T cipv_id;
  uint8_T vd_niv_left;
  uint8_T vd_niv_right;
  uint8_T cipv_lost;
  uint8_T allow_acc;
  PerceptionBEVObjectInfo cam_objects[40];
} PerceptionBEVObject;

typedef struct {
  uint64_T sync_id;
  uint8_T lane_host_count;
  LaneHost lane_host[2];
  uint8_T lane_adjacent_count;
  LaneAdjacent lane_adjacent[4];
  uint8_T road_edge_count;
  RoadEdge lane_road_edge[4];
  uint8_T estimated_width;
  uint8_T lh_crossing_left_bo;
  uint8_T lh_crossing_right_bo;
  MergeExitIndicator merge_exit_indicator;
} RoadLaneSet;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_can;
  ThrottleCommand throttle_command;
  SteerCommand steering_command;
  BrakeCommand braking_command;
  GearCommand gear_command;
  EPBCommand epb_command;
} CarControlCommandH;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_can;
  uint8_T bcm_enable;
  uint8_T turn_light_command;
  uint8_T high_beam_command;
  uint8_T dipped_beam_command;
  uint8_T emergency_light_command;
  uint8_T front_fog_lamp_command;
  uint8_T rear_fog_lamp_command;
  uint8_T brake_light_command;
  uint8_T horn_command;
  uint8_T front_wind_shield_wiper;
  uint8_T rear_wind_shield_wiper;
} CarControlCommandL;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_ins;
  real32_T acce_x;
  real32_T acce_y;
  real32_T acce_z;
  real32_T gyro_x;
  real32_T gyro_y;
  real32_T gyro_z;
  real32_T heading;
  real32_T pitch;
  real32_T roll;
  real32_T imu_temperature;
} IMUInfo;

typedef struct {
  uint64_T timestamp;
  uint8_T obj_cnt;
  uint64_T sys_reserve0;
  Barriercade barriercade[30];
} BarriercadeSet;

typedef struct {
  uint64_T time_stamp;
  uint8_T counter;
  uint32_T version;
  uint8_T processing_status;
  uint8_T processing_status2;
  uint8_T processing_status3;
  uint8_T processing_status4;
  uint8_T processing_status5;
} WorkingStatus;

typedef struct {
  MessageHeader header_struct;
  UssObjData uss_obj_data[20];
} UssObjDataStruct;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_can;
  uint8_T counter;
  uint8_T object_cnt;
  LRRObjectInfo radar_object_set[64];
} LRRObjects;

typedef struct {
  MessageHeader header_struct;
  UssDeStruct uss_de_data;
  UssCeStruct uss_ce_data;
} UssDeCeStruct;

typedef struct {
  MessageHeader header_struct;
  uint8_T psl_status;
  UssPSData uss_ps_data[20];
} UssPSDataStruct;

typedef struct {
  uint64_T time_stamp;
  uint8_T counter;
  uint8_T NCP_enable;
  uint8_T parking_out_dir_enable;
  uint8_T OCE_enable;
  uint8_T FPA_enable;
  uint8_T reverse1;
  uint64_T reverse2;
  uint64_T reverse3;
} FusionFeedbackState;

typedef struct {
  uint64_T time_stamp;
  uint8_T counter;
  uint8_T object_cnt;
  TrackedObjectInfo tracked_object_set[64];
} TrackedObjects;

typedef struct {
  uint64_T time_stamp;
  uint8_T counter;
  uint8_T status;
  real32_T x;
  real32_T y;
  uint16_T uss_point_position;
  uint8_T point_type;
  uint8_T feedback_1;
  uint8_T feedback_2;
  real32_T feedback_3;
  real32_T feedback_4;
} ControlDataFeedBack;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_slot;
  uint32_T frame_index;
  FusionParkingSlotToPlanAndControl slot_to_plan_and_control;
} FusionParkingSlotSetToPlanAndControl;

typedef struct {
  uint64_T time_stamp;
  uint64_T time_stamp_camera;
  uint32_T frame_index;
  uint8_T info_cnt;
  FusFreespaceInfo freespace_set;
} FusFreespaceSet;

typedef struct {
  uint64_T time_stamp;
  uint8_T peb_flag_set;
} PebFlagSet;

#endif                                 /* RTW_HEADER_Rte_Type_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
