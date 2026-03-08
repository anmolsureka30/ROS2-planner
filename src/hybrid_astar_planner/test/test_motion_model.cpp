#include <gtest/gtest.h>
#include "hybrid_astar_planner/motion_model.hpp"
#include "hybrid_astar_planner/types.hpp"

using namespace hybrid_astar_planner;

class MotionModelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    VehicleConfig vehicle;
    vehicle.wheelbase = 0.18;
    vehicle.length = 0.30;
    vehicle.width = 0.20;
    vehicle.rear_axle_to_back = 0.06;
    vehicle.max_steering_angle = 30.0 * M_PI / 180.0;

    MotionConfig motion;
    motion.step_size = 0.05;
    motion.num_steering_angles = 5;
    motion.allow_reverse = true;

    model_.initialize(vehicle, motion);
  }

  MotionModel model_;
};

TEST_F(MotionModelTest, PrimitivesGenerated)
{
  // 5 steering angles × 2 directions (forward + reverse) = 10
  EXPECT_EQ(model_.getPrimitives().size(), 10u);
}

TEST_F(MotionModelTest, StraightMotionForward)
{
  State start(1.0, 1.0, 0.0);
  State result = model_.applyMotion(start, Direction::FORWARD, 0.0);

  // Should move forward along x-axis
  EXPECT_NEAR(result.x, 1.05, 1e-6);
  EXPECT_NEAR(result.y, 1.0, 1e-6);
  EXPECT_NEAR(result.theta, 0.0, 1e-6);
}

TEST_F(MotionModelTest, StraightMotionReverse)
{
  State start(1.0, 1.0, 0.0);
  State result = model_.applyMotion(start, Direction::REVERSE, 0.0);

  // Should move backward along x-axis
  EXPECT_NEAR(result.x, 0.95, 1e-6);
  EXPECT_NEAR(result.y, 1.0, 1e-6);
}

TEST_F(MotionModelTest, ArcMotionChangesHeading)
{
  State start(0.0, 0.0, 0.0);
  double steering = 15.0 * M_PI / 180.0;
  State result = model_.applyMotion(start, Direction::FORWARD, steering);

  // Heading should change (left turn = positive theta change)
  EXPECT_GT(result.theta, 0.0);
  // Should still be roughly near origin
  EXPECT_NEAR(result.x, 0.05, 0.01);
}

TEST_F(MotionModelTest, TurningRadiusCorrect)
{
  // R = L / tan(delta_max) = 0.18 / tan(30°) ≈ 0.3118
  EXPECT_NEAR(model_.getMinTurningRadius(), 0.18 / std::tan(30.0 * M_PI / 180.0), 1e-4);
}

TEST_F(MotionModelTest, FootprintHasCorrectPointCount)
{
  State state(0.0, 0.0, 0.0);
  auto fp = model_.getFootprint(state);

  // Should have 16 sparse collision check points
  EXPECT_EQ(fp.points.size(), 16u);
}

TEST_F(MotionModelTest, VerticesRotateCorrectly)
{
  State state(0.0, 0.0, M_PI / 2.0);  // facing up
  auto verts = model_.getVertices(state);

  // Front-left corner should now be in negative x direction (rotated 90°)
  // Original front-left was at (front, half_w) = (0.24, 0.10)
  // After 90° rotation: (-0.10, 0.24)
  EXPECT_NEAR(verts[0].first, -0.10, 0.01);
  EXPECT_NEAR(verts[0].second, 0.24, 0.01);
}

TEST_F(MotionModelTest, AngleNormalization)
{
  EXPECT_NEAR(normalizeAngle(3.5 * M_PI), -0.5 * M_PI, 1e-10);
  EXPECT_NEAR(normalizeAngle(-3.5 * M_PI), 0.5 * M_PI, 1e-10);
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-10);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
