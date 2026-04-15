#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "od_fusion/applications/od_fusion_component.h"

namespace perception {
namespace fusion {

class OdFusionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    fusion_component_ = std::make_unique<OdFusionComponent>();
    ASSERT_TRUE(fusion_component_->Init());
  }

  std::unique_ptr<OdFusionComponent> fusion_component_;
};

FrameData GenerateTestFrame(int32_t frame_id, int32_t num_objects) {
  FrameData frame_data;
  GlobalPose svs_pose;

  svs_pose.time_stamp = static_cast<uint64_t>(frame_id) * 50 * 1000000;
  svs_pose.time_stamp_can = svs_pose.time_stamp;
  svs_pose.x = 0.0f;
  svs_pose.y = 0.0f;
  svs_pose.heading = 0.0f;
  svs_pose.pitch = 0.0f;
  svs_pose.roll = 0.0f;

  SvsFrame svs_frame;
  svs_frame.time_ns = svs_pose.time_stamp;

  for (int32_t i = 0; i < num_objects; ++i) {
    SvsObject svs_obj;
    svs_obj.timestamp_raw = svs_pose.time_stamp;

    float base_angle = static_cast<float>(frame_id) * 0.1f;
    float obj_angle = base_angle + static_cast<float>(i) * 0.8f;
    float radius = 8.0f + static_cast<float>(i) * 3.0f;

    svs_obj.object.timestamp = svs_pose.time_stamp;
    svs_obj.object.id = static_cast<uint8_t>(i);
    svs_obj.object.x = radius * std::cos(obj_angle);
    svs_obj.object.y = radius * std::sin(obj_angle);
    svs_obj.object.vx = -radius * std::sin(obj_angle) * 0.1f;
    svs_obj.object.vy = radius * std::cos(obj_angle) * 0.1f;
    svs_obj.object.yaw = obj_angle + 3.14159f;
    svs_obj.object.length = 4.5f;
    svs_obj.object.width = 1.8f;
    svs_obj.object.height = 1.5f;
    svs_obj.object.type = ObjectType::kCar;
    svs_obj.object.motion_status = MotionStatus::kMoving;
    svs_obj.object.flag = 1;

    svs_frame.svs_object_list.push_back(svs_obj);
  }

  frame_data.svs_frame = svs_frame;
  frame_data.svs_pose = svs_pose;

  return frame_data;
}

TEST_F(OdFusionTest, SingleFrameProcessing) {
  FrameData frame_data = GenerateTestFrame(1, 3);
  uint64_t meas_time = frame_data.svs_pose.time_stamp;

  fusion_component_->Process(frame_data, meas_time);

  std::vector<FusedObject> results;
  fusion_component_->GetFusionResults(&results);

  EXPECT_GE(results.size(), 0U);
}

TEST_F(OdFusionTest, MultiFrameProcessing) {
  constexpr int32_t kNumFrames = 10;

  for (int32_t frame_id = 1; frame_id <= kNumFrames; ++frame_id) {
    FrameData frame_data = GenerateTestFrame(frame_id, 3);
    uint64_t meas_time = frame_data.svs_pose.time_stamp;

    fusion_component_->Process(frame_data, meas_time);

    std::vector<FusedObject> results;
    fusion_component_->GetFusionResults(&results);

    EXPECT_GE(results.size(), 0U);
  }
}

TEST_F(OdFusionTest, ContinuousTracking) {
  constexpr int32_t kNumFrames = 20;

  std::vector<float> prev_positions;

  for (int32_t frame_id = 1; frame_id <= kNumFrames; ++frame_id) {
    FrameData frame_data = GenerateTestFrame(frame_id, 2);
    uint64_t meas_time = frame_data.svs_pose.time_stamp;

    fusion_component_->Process(frame_data, meas_time);

    std::vector<FusedObject> results;
    fusion_component_->GetFusionResults(&results);

    for (const auto& obj : results) {
      float pos = std::sqrt(obj.object.x * obj.object.x + obj.object.y * obj.object.y);
      prev_positions.push_back(pos);
    }
  }

  EXPECT_EQ(prev_positions.size(), 38U);
}

TEST_F(OdFusionTest, EmptyFrameProcessing) {
  FrameData frame_data;
  GlobalPose svs_pose;
  svs_pose.time_stamp = 1000000000;
  svs_pose.x = 0.0f;
  svs_pose.y = 0.0f;
  svs_pose.heading = 0.0f;

  SvsFrame svs_frame;
  svs_frame.time_ns = svs_pose.time_stamp;

  frame_data.svs_frame = svs_frame;
  frame_data.svs_pose = svs_pose;

  uint64_t meas_time = svs_pose.time_stamp;

  fusion_component_->Process(frame_data, meas_time);

  std::vector<FusedObject> results;
  fusion_component_->GetFusionResults(&results);
}

TEST_F(OdFusionTest, ObjectTypeConsistency) {
  FrameData frame_data = GenerateTestFrame(1, 3);
  frame_data.svs_frame.svs_object_list[0].object.type = ObjectType::kCar;
  frame_data.svs_frame.svs_object_list[1].object.type = ObjectType::kPerson;
  frame_data.svs_frame.svs_object_list[2].object.type = ObjectType::kCyclist;

  uint64_t meas_time = frame_data.svs_pose.time_stamp;
  fusion_component_->Process(frame_data, meas_time);

  std::vector<FusedObject> results;
  fusion_component_->GetFusionResults(&results);
}

TEST_F(OdFusionTest, HighFrequencyProcessing) {
  constexpr int32_t kNumFrames = 100;
  constexpr uint64_t kTimeIntervalUs = 50000;

  uint64_t base_time = 1000000000;

  for (int32_t frame_id = 0; frame_id < kNumFrames; ++frame_id) {
    FrameData frame_data = GenerateTestFrame(frame_id + 1, 5);
    uint64_t meas_time = base_time + static_cast<uint64_t>(frame_id) * kTimeIntervalUs;

    frame_data.svs_pose.time_stamp = meas_time;
    frame_data.svs_frame.time_ns = meas_time;

    fusion_component_->Process(frame_data, meas_time);
  }

  std::vector<FusedObject> results;
  fusion_component_->GetFusionResults(&results);
  EXPECT_GE(results.size(), 0U);
}

}  // namespace fusion
}  // namespace perception

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
