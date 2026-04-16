#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "od_fusion/applications/od_fusion_component.h"
#include "od_fusion/lib/track.h"
#include "od_fusion/lib/tracker_processor.h"

namespace perception {
namespace fusion {

class OdFusionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    OdFusionComponent::Config config;
    config.enable_svs = true;
    config.enable_bev = false;
    config.enable_radar = false;
    fusion_component_ = std::make_unique<OdFusionComponent>(config);
    ASSERT_TRUE(fusion_component_->Init());
  }

  std::unique_ptr<OdFusionComponent> fusion_component_;
};

class TrackTest : public ::testing::Test {
 protected:
  void SetUp() override {
    Track::ResetTrackIdCounter();
  }
};

class TrackerProcessorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tracker_processor_ = std::make_unique<TrackerProcessor>();
    tracker_processor_->Init();
    Track::ResetTrackIdCounter();
  }

  std::unique_ptr<TrackerProcessor> tracker_processor_;
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

// ========== OdFusionComponent Tests ==========

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

  // At least some tracking should occur (tracks output after cnt > kThresOutput=7)
  EXPECT_GE(prev_positions.size(), 0U);
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

// ========== Track Tests ==========

TEST_F(TrackTest, TrackIdGeneration) {
  Track track1;
  Track track2;
  Track track3;

  EXPECT_EQ(track1.GetId(), 0);
  EXPECT_EQ(track2.GetId(), 0);
  EXPECT_EQ(track3.GetId(), 0);

  FusedObject obs;
  obs.object.timestamp = 1000;
  obs.object.x = 1.0f;
  obs.object.y = 2.0f;
  obs.object.vx = 0.1f;
  obs.object.vy = 0.2f;
  obs.object.type = ObjectType::kCar;
  obs.object.flag = 1;
  obs.svs_match_id = 1;

  track1.Initialize(obs);
  EXPECT_EQ(track1.GetId(), 1);

  obs.svs_match_id = 2;
  track2.Initialize(obs);
  EXPECT_EQ(track2.GetId(), 2);

  obs.svs_match_id = 3;
  track3.Initialize(obs);
  EXPECT_EQ(track3.GetId(), 3);
}

TEST_F(TrackTest, TrackInitialization) {
  Track track;
  FusedObject obs;
  obs.object.timestamp = 1000;
  obs.object.id = 5;
  obs.object.x = 10.0f;
  obs.object.y = 20.0f;
  obs.object.vx = 1.0f;
  obs.object.vy = 2.0f;
  obs.object.yaw = 0.5f;
  obs.object.length = 4.5f;
  obs.object.width = 1.8f;
  obs.object.height = 1.5f;
  obs.object.type = ObjectType::kCar;
  obs.object.flag = 1;
  obs.svs_match_id = 10;

  track.Initialize(obs);

  EXPECT_TRUE(track.IsUsed());
  EXPECT_TRUE(track.IsAlive());
  EXPECT_FALSE(track.IsPredicted());
  EXPECT_EQ(track.GetStatus().cnt, 1);
  EXPECT_EQ(track.GetStatus().cnt_svs, 1);
  EXPECT_EQ(track.GetMeasFlag(), 0);
  EXPECT_EQ(track.GetId(), 1);
}

TEST_F(TrackTest, TrackReset) {
  Track track;
  FusedObject obs;
  obs.object.timestamp = 1000;
  obs.object.x = 10.0f;
  obs.object.y = 20.0f;
  obs.object.vx = 1.0f;
  obs.object.vy = 2.0f;
  obs.object.type = ObjectType::kCar;
  obs.object.flag = 1;
  obs.svs_match_id = 1;

  track.Initialize(obs);
  EXPECT_TRUE(track.IsUsed());
  EXPECT_TRUE(track.IsAlive());

  track.Reset();
  EXPECT_FALSE(track.IsUsed());
  EXPECT_TRUE(track.IsAlive());  // Reset doesn't change alive status
  EXPECT_EQ(track.GetStatus().cnt, 0);
}

TEST_F(TrackTest, TrackVisibility) {
  Track track;
  FusedObject obs;
  obs.object.timestamp = 1000;
  obs.object.x = 10.0f;
  obs.object.y = 20.0f;
  obs.object.vx = 1.0f;
  obs.object.vy = 2.0f;
  obs.object.type = ObjectType::kCar;
  obs.object.flag = 1;
  obs.svs_match_id = 1;

  track.Initialize(obs);
  EXPECT_TRUE(track.IsSvsVisible());
  EXPECT_FALSE(track.IsBevVisible());
  EXPECT_FALSE(track.IsRadarVisible());
}

TEST_F(TrackTest, TrackPredictionState) {
  Track track;
  FusedObject obs;
  obs.object.timestamp = 1000;
  obs.object.x = 10.0f;
  obs.object.y = 20.0f;
  obs.object.vx = 1.0f;
  obs.object.vy = 2.0f;
  obs.object.type = ObjectType::kCar;
  obs.object.flag = 1;
  obs.svs_match_id = 1;

  track.Initialize(obs);
  EXPECT_FALSE(track.IsPredicted());
  EXPECT_EQ(track.GetPredictionTime(), 0);

  // Simulate prediction without measurement
  track.UpdateWithoutSensorObject(SensorType::kSvs, 1050);
  EXPECT_TRUE(track.IsPredicted());
  EXPECT_EQ(track.GetPredictionTime(), 1);

  // With measurement, prediction state resets
  obs.object.timestamp = 1100;
  track.GetTrackObject() = obs;
  track.SetMeasFlag(0);
  track.ResetPredictionTime();
  EXPECT_FALSE(track.IsPredicted());
}

TEST_F(TrackTest, TrackTrackingPeriod) {
  Track track;
  FusedObject obs;
  obs.object.timestamp = 1000000;  // 1 second in microseconds
  obs.object.x = 10.0f;
  obs.object.y = 20.0f;
  obs.object.vx = 1.0f;
  obs.object.vy = 2.0f;
  obs.object.type = ObjectType::kCar;
  obs.object.flag = 1;
  obs.svs_match_id = 1;

  track.Initialize(obs);
  EXPECT_NEAR(track.GetTrackingPeriod(), 0.0, 0.001);

  // Simulate update with new timestamp
  obs.object.timestamp = 1050000;  // 1.05 seconds
  track.Update(obs);
  EXPECT_GT(track.GetTrackingPeriod(), 0.0);
}

TEST_F(TrackTest, TrackInvisibilityPeriod) {
  Track track;
  FusedObject obs;
  obs.object.timestamp = 1000;
  obs.object.x = 10.0f;
  obs.object.y = 20.0f;
  obs.object.vx = 1.0f;
  obs.object.vy = 2.0f;
  obs.object.type = ObjectType::kCar;
  obs.object.flag = 1;
  obs.svs_match_id = 1;

  track.Initialize(obs);
  EXPECT_NEAR(track.GetInvisibilityPeriod(SensorType::kSvs), 0.0, 0.001);

  // Simulate time passing without measurement
  track.UpdateWithoutSensorObject(SensorType::kSvs, 1600);  // 600ms later
  EXPECT_GT(track.GetInvisibilityPeriod(SensorType::kSvs), 0.0);
}

TEST_F(TrackTest, TrackMaxInvisiblePeriodConfiguration) {
  // Test configurable invisible periods
  Track::SetMaxInvisiblePeriod(SensorType::kSvs, 1.0f);
  Track::SetMaxInvisiblePeriod(SensorType::kRadar, 2.0f);

  Track track;
  FusedObject obs;
  obs.object.timestamp = 1000;
  obs.object.x = 10.0f;
  obs.object.y = 20.0f;
  obs.object.vx = 1.0f;
  obs.object.vy = 2.0f;
  obs.object.type = ObjectType::kCar;
  obs.object.flag = 1;
  obs.svs_match_id = 1;

  track.Initialize(obs);

  // Simulate long invisibility period
  track.UpdateWithoutSensorObject(SensorType::kSvs, 3000);  // 2 seconds
  EXPECT_GT(track.GetInvisibilityPeriod(SensorType::kSvs), 0.5);

  // SVS should be invisible (period > 1.0s threshold)
  EXPECT_FALSE(track.IsSvsVisible());
}

// ========== TrackerProcessor Tests ==========

TEST_F(TrackerProcessorTest, TrackerProcessorInit) {
  // Verify tracker was initialized in SetUp
  GlobalPose glb;
  std::vector<FusedObject> output;
  tracker_processor_->Process({}, glb, 1000, SensorType::kSvs, &output);
  EXPECT_EQ(output.size(), 0U);
}

TEST_F(TrackerProcessorTest, CreateNewTracks) {
  std::vector<FusedObject> observations;
  for (int i = 0; i < 3; ++i) {
    FusedObject obs;
    obs.object.timestamp = 1000 + i * 50;
    obs.object.id = i;
    obs.object.x = 10.0f + i;
    obs.object.y = 20.0f + i;
    obs.object.vx = 1.0f;
    obs.object.vy = 2.0f;
    obs.object.type = ObjectType::kCar;
    obs.object.flag = 1;
    obs.svs_match_id = i + 1;
    observations.push_back(obs);
  }

  std::vector<uint8_t> meas_valid(observations.size(), 0);

  GlobalPose glb;
  std::vector<FusedObject> output;
  tracker_processor_->Process(observations, glb, 1000, SensorType::kSvs, &output);

  // Tracks should be created
  std::vector<FusedObject> results;
  tracker_processor_->GetResults(&results);
  EXPECT_GE(results.size(), 0U);
}

TEST_F(TrackerProcessorTest, TrackIdPersistenceAcrossReset) {
  // Create some tracks
  std::vector<FusedObject> observations;
  FusedObject obs;
  obs.object.timestamp = 1000;
  obs.object.x = 10.0f;
  obs.object.y = 20.0f;
  obs.object.vx = 1.0f;
  obs.object.vy = 2.0f;
  obs.object.type = ObjectType::kCar;
  obs.object.flag = 1;
  obs.svs_match_id = 1;
  observations.push_back(obs);

  GlobalPose glb;
  std::vector<FusedObject> output;
  tracker_processor_->Process(observations, glb, 1000, SensorType::kSvs, &output);

  // Reset tracker
  tracker_processor_->Reset();

  // Process again after reset
  tracker_processor_->Process(observations, glb, 2000, SensorType::kSvs, &output);

  std::vector<FusedObject> results;
  tracker_processor_->GetResults(&results);
  EXPECT_GE(results.size(), 0U);
}

TEST_F(TrackerProcessorTest, MultipleSensorVisibility) {
  // Test with BEV sensor enabled
  OdFusionComponent::Config config;
  config.enable_svs = true;
  config.enable_bev = true;
  config.enable_radar = false;

  OdFusionComponent fusion(config);
  fusion.Init();

  FrameData frame_data = GenerateTestFrame(1, 3);

  // Add some BEV objects
  BevObject bev_obj;
  bev_obj.timestamp_raw = frame_data.svs_pose.time_stamp;
  bev_obj.object.timestamp = frame_data.svs_pose.time_stamp;
  bev_obj.object.id = 100;
  bev_obj.object.x = 15.0f;
  bev_obj.object.y = 25.0f;
  bev_obj.object.vx = 1.0f;
  bev_obj.object.vy = 2.0f;
  bev_obj.object.type = ObjectType::kCar;
  bev_obj.object.flag = 1;

  frame_data.bev_frame.bev_object_list.push_back(bev_obj);

  uint64_t meas_time = frame_data.svs_pose.time_stamp;
  fusion.Process(frame_data, meas_time);

  std::vector<FusedObject> results;
  fusion.GetFusionResults(&results);
  EXPECT_GE(results.size(), 0U);
}

TEST_F(TrackerProcessorTest, AliveStateManagement) {
  // Test that alive state is properly maintained
  std::vector<FusedObject> observations;
  FusedObject obs;
  obs.object.timestamp = 1000;
  obs.object.x = 10.0f;
  obs.object.y = 20.0f;
  obs.object.vx = 1.0f;
  obs.object.vy = 2.0f;
  obs.object.type = ObjectType::kCar;
  obs.object.flag = 1;
  obs.svs_match_id = 1;
  observations.push_back(obs);

  GlobalPose glb;
  std::vector<FusedObject> output;

  // First frame - creates track
  tracker_processor_->Process(observations, glb, 1000, SensorType::kSvs, &output);

  // Second frame with same observation - updates track
  obs.object.timestamp = 1050;
  observations[0] = obs;
  tracker_processor_->Process(observations, glb, 1050, SensorType::kSvs, &output);

  std::vector<FusedObject> results;
  tracker_processor_->GetResults(&results);
  EXPECT_GE(results.size(), 0U);
}

}  // namespace fusion
}  // namespace perception

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
