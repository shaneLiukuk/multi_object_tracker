#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#if 1
#define HCOUT std::cout
#else
#define HCOUT  // std::cout
#endif
#define H_DEBUG_R "\033[31m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_G "\033[32m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_B "\033[34m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"
#define H_DEBUG_Y "\033[33m[Debug] " << __FUNCTION__ << __LINE__ << "\033[0m"

#define HDEBUG_R "\033[31m[Debug] \033[0m"
#define HDEBUG_G "\033[32m[Debug] \033[0m"
#define HDEBUG_B "\033[34m[Debug] \033[0m"
#define HDEBUG_Y "\033[33m[Debug] \033[0m"

#define H_R "\033[31m"
#define H_G "\033[32m"
#define H_B "\033[34m"
#define H_Y "\033[33m"
#define H_P "\033[35m"
#define H_C "\033[36m"
#define H_0 "\033[0m"


namespace perception {
namespace fusion {

// 最大观测/跟踪数量配置
constexpr uint8_t kMaxObsSvs = 64;
constexpr uint8_t kMaxObsBev = 20;
constexpr uint8_t kMaxObsRadar = 64;
constexpr uint8_t kMaxObsFuse = 64;
constexpr uint8_t kTrackDepth = 30;
constexpr uint8_t kTrackWidth = 64;
constexpr uint8_t kMaxOutputTracks = 64;
constexpr uint8_t kLen1st = 3;

// 跟踪状态阈值配置
constexpr int32_t kThresStable = 3;
constexpr int32_t kThresUnstable = 2;
constexpr int32_t kThresLost = 10;
constexpr int32_t kThresOutput = 7;
constexpr int32_t kThresIdAssign = 20;

// 滤波噪声与权重配置
constexpr float kProcessNoisePos = 0.000001f;
constexpr float kProcessNoiseVel = 0.00000f;
constexpr float kMeasurementNoiseRx = 0.00000f;
constexpr float kMeasurementNoiseRy = 0.000001f;
constexpr float kMeasurementNoiseRvx = 0.000001f;
constexpr float kMeasurementNoiseRvy = 0.000001f;
constexpr float kDistGate = 20.0f;
constexpr float kPosWeight = 1.0f;
constexpr float kVelWeight = 0.1f;

// 传感器范围配置
constexpr float kSensorRangeYMin = -8.0f;
constexpr float kSensorRangeYMax = 20.0f;
constexpr float kSensorRangeXMin = -10.0f;
constexpr float kSensorRangeXMax = 10.0f;

// 静态目标判断配置
constexpr float kStaticCntLow = 20.0f;
constexpr float kStaticDisThres = 15.0f;
constexpr float kVelocityThreshold = 1e-4f;

// 时间、数学常量配置
constexpr float kPi = 3.14159265358979f;
constexpr float kTimeDiffMin = 0.05f;
constexpr float kTimeDiffMax = 0.5f;
constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
const double kMAX = std::numeric_limits<double>::max();
const double kMIN = std::numeric_limits<double>::min();

#define M_2PI 6.28318530717958647693  /* 2*pi */
#define M_PI 3.14159265358979323846   /* pi */
#define M_PI_2 1.57079632679489661923 /* pi/2 */
#define M_PI_4 0.78539816339744830962 /* pi/4 */
#define M_1_PI 0.31830988618379067154 /* 1/pi */
#define M_2_PI 0.63661977236758134308 /* 2/pi */

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*180 / M_PI)
#endif
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*M_PI / 180.0)
#endif

// 匹配代价配置
constexpr float kCostThreshold = 4.0f;
constexpr float kIdMatchCost = 0.01f;

// 重叠跟踪器去除配置
constexpr float kOverlapDistThreshold = 5.0f;
constexpr float kOverlapIoUThreshold = 0.3f;

}  // namespace fusion
}  // namespace perception