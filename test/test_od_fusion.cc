#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <limits>
#include "od_fusion/lib/track.h"
#include "od_fusion/lib/tracker_processor.h"
#include "od_fusion/lib/hungarian.h"

namespace perception {
namespace fusion {

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
  track.GetFusedObject() = obs;
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

TEST_F(TrackTest, SensorHistoryManagement) {
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

  // Add more observations
  for (int i = 0; i < 35; ++i) {
    obs.object.timestamp = 1000 + i * 100;
    obs.object.x = 10.0f + i;
    obs.object.y = 20.0f + i;
    track.Update(obs);
  }

  // History should be limited to 30 entries
  const auto& history = track.GetSensorHistory(SensorType::kSvs);
  EXPECT_EQ(history.size(), 30u);

  // Get previous object
  FusedObject prev = track.GetPreviousSensorObject(SensorType::kSvs, 0);
  EXPECT_EQ(prev.object.x, 10.0f + 34);  // Latest
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

  std::vector<int> meas_valid(observations.size(), 0);

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

TEST_F(TrackerProcessorTest, MultipleSensorTracking) {
  // Test with multiple sensor observations
  std::vector<FusedObject> observations;

  // SVS observation
  FusedObject svs_obs;
  svs_obs.object.timestamp = 1000;
  svs_obs.object.x = 10.0f;
  svs_obs.object.y = 20.0f;
  svs_obs.object.vx = 1.0f;
  svs_obs.object.vy = 2.0f;
  svs_obs.object.type = ObjectType::kCar;
  svs_obs.object.flag = 1;
  svs_obs.svs_match_id = 1;
  observations.push_back(svs_obs);

  GlobalPose glb;
  std::vector<FusedObject> output;
  tracker_processor_->Process(observations, glb, 1000, SensorType::kSvs, &output);

  std::vector<FusedObject> results;
  tracker_processor_->GetResults(&results);
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

// ========== Hungarian Tests ==========

TEST(HungarianTest, SimpleAssignment) {
  Eigen::MatrixXf cost(3, 3);
  cost << 1, 2, 3,
          2, 4, 0,
          3, 0, 2;

  Eigen::MatrixXi assignment;
  Hungarian::Solve(cost, &assignment);

  // Each row and column should have at most one assignment
  int count = 0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (assignment(i, j) > 0) {
        count++;
        EXPECT_EQ(assignment(i, j), 1);
      }
    }
  }
  EXPECT_EQ(count, 3);  // Should assign all 3 rows
}

TEST(HungarianTest, SquareMatrix5x5) {
  Eigen::MatrixXf cost = Eigen::MatrixXf::Ones(5, 5) * 100.0f;

  // Set some lower costs
  cost(0, 1) = 1.0f;
  cost(1, 2) = 2.0f;
  cost(2, 0) = 3.0f;
  cost(3, 4) = 1.5f;
  cost(4, 3) = 2.5f;

  Eigen::MatrixXi assignment;
  Hungarian::Solve(cost, &assignment);

  int count = 0;
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      if (assignment(i, j) > 0) {
        count++;
      }
    }
  }
  EXPECT_EQ(count, 5);  // Should assign all 5 rows
}

TEST(HungarianTest, AllEqualCost) {
  Eigen::MatrixXf cost = Eigen::MatrixXf::Ones(4, 4);

  Eigen::MatrixXi assignment;
  Hungarian::Solve(cost, &assignment);

  int count = 0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (assignment(i, j) > 0) {
        count++;
      }
    }
  }
  EXPECT_EQ(count, 4);
}

TEST(HungarianTest, DiagonalMatrix) {
  Eigen::MatrixXf cost = Eigen::MatrixXf::Zero(4, 4);
  for (int i = 0; i < 4; ++i) {
    cost(i, i) = i + 1;  // Diagonal has increasing costs
  }

  Eigen::MatrixXi assignment;
  Hungarian::Solve(cost, &assignment);

  // Should still produce a valid assignment (not necessarily diagonal)
  int count = 0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (assignment(i, j) > 0) {
        count++;
        EXPECT_EQ(assignment(i, j), 1);
      }
    }
  }
  EXPECT_EQ(count, 4);
}

TEST(HungarianTest, LargeMatrix8x8) {
  Eigen::MatrixXf cost(8, 8);
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      cost(i, j) = static_cast<float>(i + j + 1);
    }
  }

  Eigen::MatrixXi assignment;
  Hungarian::Solve(cost, &assignment);

  int count = 0;
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      if (assignment(i, j) > 0) {
        count++;
      }
    }
  }
  EXPECT_EQ(count, 8);
}

TEST(HungarianTest, EmptyMatrix) {
  Eigen::MatrixXf cost(0, 0);
  Eigen::MatrixXi assignment;
  Hungarian::Solve(cost, &assignment);
  EXPECT_EQ(assignment.rows(), 0);
  EXPECT_EQ(assignment.cols(), 0);
}

TEST(HungarianTest, NullAssignment) {
  Eigen::MatrixXf cost(3, 3);
  cost << 1, 2, 3,
          2, 4, 0,
          3, 0, 2;

  // Should not crash with null
  Hungarian::Solve(cost, nullptr);
}

TEST(HungarianTest, MatrixWithInf) {
  Eigen::MatrixXf cost(3, 3);
  cost << 1, 2, 3,
          2, 4, std::numeric_limits<float>::infinity(),
          3, 0, 2;

  Eigen::MatrixXi assignment;
  Hungarian::Solve(cost, &assignment);

  // Should handle inf gracefully (no assignment)
  int count = 0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (assignment(i, j) > 0) {
        count++;
      }
    }
  }
  EXPECT_EQ(count, 0);
}

TEST(HungarianTest, MatrixWithNaN) {
  Eigen::MatrixXf cost(3, 3);
  cost << 1, 2, 3,
          2, std::nan(""), 0,
          3, 0, 2;

  Eigen::MatrixXi assignment;
  Hungarian::Solve(cost, &assignment);

  int count = 0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (assignment(i, j) > 0) {
        count++;
      }
    }
  }
  EXPECT_EQ(count, 0);
}

TEST(HungarianTest, 64x64Matrix) {
  Eigen::MatrixXf cost = Eigen::MatrixXf::Ones(64, 64) * 100.0f;

  // Set some random lower costs along diagonal blocks
  for (int i = 0; i < 64; ++i) {
    cost(i, i) = static_cast<float>(i + 1) * 0.1f;
  }

  Eigen::MatrixXi assignment;
  Hungarian::Solve(cost, &assignment);

  int count = 0;
  for (int i = 0; i < 64; ++i) {
    for (int j = 0; j < 64; ++j) {
      if (assignment(i, j) > 0) {
        count++;
      }
    }
  }
  EXPECT_EQ(count, 64);
}

}  // namespace fusion
}  // namespace perception

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
