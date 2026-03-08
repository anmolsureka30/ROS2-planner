#include <gtest/gtest.h>
#include "hybrid_astar_planner/cost_function.hpp"
#include "hybrid_astar_planner/types.hpp"

using namespace hybrid_astar_planner;

class CostFunctionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    CostConfig cost;
    cost.steering_penalty = 1.5;
    cost.reversing_penalty = 2.0;
    cost.steering_change_penalty = 1.5;
    cost.direction_switch_penalty = 10.0;
    cost.non_straight_penalty = 0.03;
    cost.change_direction_penalty = 0.05;
    cost.cost_penalty_alpha = 1.5;

    VehicleConfig vehicle;
    vehicle.max_steering_angle = 30.0 * M_PI / 180.0;

    cf_.initialize(cost, vehicle, 0.05);
  }

  CostFunction cf_;
};

TEST_F(CostFunctionTest, StraightForwardHasBaseCost)
{
  State current(0, 0, 0);
  current.direction = Direction::FORWARD;
  current.steering = 0.0;

  State neighbor(0.05, 0, 0);
  neighbor.direction = Direction::FORWARD;
  neighbor.steering = 0.0;

  double cost = cf_.computeCost(current, neighbor, 0.0);
  // Base cost = step_size = 0.05
  // No penalties applied for straight forward motion
  EXPECT_NEAR(cost, 0.05, 0.01);
}

TEST_F(CostFunctionTest, ReverseMotionCostsMore)
{
  State current(0, 0, 0);
  current.direction = Direction::FORWARD;
  current.steering = 0.0;

  State forward_neighbor(0.05, 0, 0);
  forward_neighbor.direction = Direction::FORWARD;
  forward_neighbor.steering = 0.0;

  State reverse_neighbor(-0.05, 0, 0);
  reverse_neighbor.direction = Direction::REVERSE;
  reverse_neighbor.steering = 0.0;

  double forward_cost = cf_.computeCost(current, forward_neighbor, 0.0);
  double reverse_cost = cf_.computeCost(current, reverse_neighbor, 0.0);

  EXPECT_GT(reverse_cost, forward_cost);
}

TEST_F(CostFunctionTest, SteeringPenaltyApplied)
{
  State current(0, 0, 0);
  current.direction = Direction::FORWARD;
  current.steering = 0.0;

  State straight(0.05, 0, 0);
  straight.direction = Direction::FORWARD;
  straight.steering = 0.0;

  State turning(0.05, 0.01, 0.1);
  turning.direction = Direction::FORWARD;
  turning.steering = 15.0 * M_PI / 180.0;

  double straight_cost = cf_.computeCost(current, straight, 0.0);
  double turning_cost = cf_.computeCost(current, turning, 0.0);

  EXPECT_GT(turning_cost, straight_cost);
}

TEST_F(CostFunctionTest, DirectionSwitchPenalty)
{
  State current(0, 0, 0);
  current.direction = Direction::FORWARD;
  current.steering = 0.0;

  State switch_dir(0.05, 0, 0);
  switch_dir.direction = Direction::REVERSE;
  switch_dir.steering = 0.0;

  double cost = cf_.computeCost(current, switch_dir, 0.0);

  // Should include direction_switch_penalty = 10.0
  EXPECT_GT(cost, 10.0);
}

TEST_F(CostFunctionTest, CostmapCostIncreasesTraversalCost)
{
  State current(0, 0, 0);
  current.direction = Direction::FORWARD;
  current.steering = 0.0;

  State neighbor(0.05, 0, 0);
  neighbor.direction = Direction::FORWARD;
  neighbor.steering = 0.0;

  double cost_free = cf_.computeCost(current, neighbor, 0.0);
  double cost_near_obstacle = cf_.computeCost(current, neighbor, 0.5);

  EXPECT_GT(cost_near_obstacle, cost_free);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
