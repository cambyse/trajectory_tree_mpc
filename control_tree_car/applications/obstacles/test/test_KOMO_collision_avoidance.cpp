#include <chrono>

#include "common/komo_problems.h"

#include <gtest/gtest.h>


TEST(ProbabilityFusion, DISABLED_OneObstacle2Branches)
{
  std::vector<Obstacle> obstacles;
  obstacles.push_back(Obstacle{arr(), 0.1});

  std::vector<std::vector<bool>> activities;
  auto ps = fuse_probabilities(obstacles, true, activities);

  EXPECT_EQ(std::vector<double>({0.1, 0.9}), ps);
}

TEST(ProbabilityFusion, DISABLED_TwoObstacle4Branches)
{
  std::vector<Obstacle> obstacles;
  obstacles.push_back(Obstacle{arr(), 0.1});
  obstacles.push_back(Obstacle{arr(), 0.5});

  std::vector<std::vector<bool>> activities;
  const auto ps = fuse_probabilities(obstacles, true, activities);

  EXPECT_EQ(std::vector<double>({0.05, 0.5*(1.0 - 0.1), 0.05, 1.0 - 0.05 * 2 - 0.5*(1.0 - 0.1)}), ps);
}

//TEST_F(KomoJointTest, scenario_1)
//{
//  auto pb = create_scenario_1(0.4);

//  plan(pb, true);
//  //plan(pb, false);
//}

TEST_F(KomoJointNTest1Obstacle, scenario_1)
{
  auto pb = create_scenario_1(0.4);

  plan(pb, true);
  //plan(pb, false);
}

TEST_F(KomoDecTest1Obstacle, scenario_1)
{
  auto pb = create_scenario_1(0.4);

  plan(pb, true);
  //plan(pb, false);
}

TEST_F(KomoDecTest1Obstacle, DISABLED_scenario_1_bis)
{
  auto pb = create_scenario_1(0.8);

  plan(pb, false, true);

  std::cout << "----------" << std::endl;

  //pb.odo->pose.pose.position.x += 3;

  //plan(pb, false, true);

//  std::cout << "----------" << std::endl;

//  pb.odo->pose.pose.position.x += 3;

//  plan(pb, true);
}


TEST_F(KomoJointNTest2Obstacles, scenario_2)
{
  auto pb = create_scenario_2();

  plan(pb, true);
}

TEST_F(KomoDecTest2Obstacles, scenario_2)
{
  auto pb = create_scenario_2();

  plan(pb, true);

//  pb.odo->pose.pose.position.x += 2;

//  plan(pb, true);

//  pb.odo->pose.pose.position.x += 2;

//  plan(pb, true);

//  pb.odo->pose.pose.position.x += 2;

//  plan(pb, true);

////  pb.odo->pose.pose.position.x += 2;

////  plan(pb, true);
}

TEST_F(KomoJointNTest3Obstacles, scenario_3)
{
  auto pb = create_scenario_3();

  plan(pb, true);
}

TEST_F(KomoDecTest3Obstacles, scenario_3)
{
  auto pb = create_scenario_3();

  plan(pb, true);
  //plan(pb, false);
}

TEST_F(KomoJointNTest4Obstacles, scenario_4)
{
  auto pb = create_scenario_4();

  plan(pb, true);
}

TEST_F(KomoDecTest4Obstacles, scenario_4)
{
  auto pb = create_scenario_4();

  plan(pb, true);
  //plan(pb, false);
}

TEST_F(KomoJointNTest5Obstacles, scenario_5)
{
  auto pb = create_scenario_5();

  plan(pb, true);
}

TEST_F(KomoDecTest5Obstacles, scenario_5)
{
  auto pb = create_scenario_5();

  plan(pb, true);
  //plan(pb, false);
}

TEST_F(KomoDecTestLinear1Obstacle, DISABLED_scenario_1)
{
  auto pb = create_scenario_1(0.02);

  plan(pb, true);
  plan(pb, false);
}

TEST_F(KomoDecTestLinear2Obstacle, DISABLED_scenario_2)
{
  auto pb = create_scenario_2();

  plan(pb, true);
  plan(pb, false);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

