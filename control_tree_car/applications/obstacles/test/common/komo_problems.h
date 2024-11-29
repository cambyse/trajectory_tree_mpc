#include <common/behavior_manager.h>

#include <komo/obstacle_avoidance_linear.h>
#include <komo/obstacle_avoidance_tree.h>
#include <komo/obstacle_avoidance_tree_n.h>
#include <komo/obstacle_avoidance_dec.h>
#include <komo/trajectory_plot.h>

#include <gtest/gtest.h>

nav_msgs::Odometry::Ptr create_odo(double x, double y, double vx);
std_msgs::Float32::Ptr create_desired_speed(double v);
visualization_msgs::MarkerArray::Ptr create_obstacles(double x, double y, double alpha);

struct Scenario
{
  nav_msgs::Odometry::Ptr odo;
  std_msgs::Float32::Ptr desired_velocity;
  visualization_msgs::MarkerArray::Ptr obstacles;
};

Scenario create_scenario_1(double p);
Scenario create_scenario_2();
Scenario create_scenario_3();
Scenario create_scenario_4();
Scenario create_scenario_5();

template<typename T>
void plan_impl(const Scenario & scenario, BehaviorManager& manager, T& behavior, bool _plot, bool _plot_debug)
{
  manager.odometry_callback(scenario.odo);

  behavior->desired_speed_callback(scenario.desired_velocity);

  behavior->obstacle_callback(scenario.obstacles);

  auto plotter = [&](const auto&...)
  {
      static int n = 0;

      PlotAxis axis{std::to_string(n) + " x/y", "[-4:3]"};
      plot(manager.get_trajectories(), scenario.obstacles, axis);

      ++n;
  };

  if(_plot_debug)
  {
    behavior->set_optim_callback(plotter);
  }

  manager.plan();

  if(_plot)
  {
    plotter();
  }
}

class KomoJointTest : public ::testing::Test
{
public:
  void SetUp()
  {
    ros::Time::init();

    behavior = std::make_shared<ObstacleAvoidanceTree>(manager, std::string{"/home/camille/git/catkin_ws/src/icra_2021/control_tree_car/data/LGP-real-time.g"}, 4);

    manager.register_behavior("collision_avoidance", behavior);
    manager.set_current_behavior("collision_avoidance");
  }

  void plan(const Scenario & scenario, bool plot, bool plot_debug = false)
  {
    plan_impl(scenario, manager, behavior, plot, plot_debug);
  }

  BehaviorManager manager;
  std::shared_ptr<ObstacleAvoidanceTree> behavior;
};

class KomoJointNTest : public ::testing::Test
{
public:
  void SetUp()
  {
    ros::Time::init();

    behavior = std::make_shared<ObstacleAvoidanceTreeN>(manager, std::string{"/home/camille/git/catkin_ws/src/icra_2021/control_tree_car/data/LGP-real-time.g"}, n_obstacles, 5, 4);

    manager.register_behavior("collision_avoidance", behavior);
    manager.set_current_behavior("collision_avoidance");
  }

  void plan(const Scenario & scenario, bool plot, bool plot_debug = false)
  {
    plan_impl(scenario, manager, behavior, plot, plot_debug);
  }

  BehaviorManager manager;
  std::shared_ptr<ObstacleAvoidanceTreeN> behavior;
  int n_obstacles = 1;
};

class KomoJointNTest1Obstacle : public KomoJointNTest
{
   public:
    KomoJointNTest1Obstacle()
    {
        n_obstacles = 1;
    }
};

class KomoJointNTest2Obstacles : public KomoJointNTest
{
   public:
    KomoJointNTest2Obstacles()
    {
        n_obstacles = 2;
    }
};

class KomoJointNTest3Obstacles : public KomoJointNTest
{
   public:
    KomoJointNTest3Obstacles()
    {
        n_obstacles = 3;
    }
};

class KomoJointNTest4Obstacles : public KomoJointNTest
{
   public:
    KomoJointNTest4Obstacles()
    {
        n_obstacles = 4;
    }
};

class KomoJointNTest5Obstacles : public KomoJointNTest
{
   public:
    KomoJointNTest5Obstacles()
    {
        n_obstacles = 5;
    }
};

class KomoSimpleForkTest : public ::testing::Test
{
public:
  void SetUp()
  {
    ros::Time::init();

    behavior = std::make_shared<ObstacleAvoidanceTreeN>(manager, std::string{"/home/camille/git/catkin_ws/src/icra_2021/control_tree_car/data/LGP-real-time.g"}, 1, 2, 4);

    manager.register_behavior("collision_avoidance", behavior);
    manager.set_current_behavior("collision_avoidance");
  }

  void plan(const Scenario & scenario, bool plot, bool plot_debug = false)
  {
    plan_impl(scenario, manager, behavior, plot, plot_debug);
  }

  BehaviorManager manager;
  std::shared_ptr<ObstacleAvoidanceTreeN> behavior;
  int n_obstacles = 1;
};

class KomoDecTest : public ::testing::Test
{
public:
  void SetUp()
  {
    ros::Time::init();

    behavior = std::make_shared<ObstacleAvoidanceDec>(manager, std::string{"/home/camille/git/catkin_ws/src/icra_2021/control_tree_car/data/LGP-real-time.g"}, n_obstacles, tree, 3.5, 10, 4);

    manager.register_behavior("collision_avoidance", behavior);
    manager.set_current_behavior("collision_avoidance");
  }

  void plan(const Scenario & scenario, bool plot, bool plot_debug = false)
  {
    plan_impl(scenario, manager, behavior, plot, plot_debug);
  }

  BehaviorManager manager;
  std::shared_ptr<ObstacleAvoidanceDec> behavior;
  int n_obstacles = 2;
  bool tree = true;
};

class KomoDecTest1Obstacle : public KomoDecTest
{
   public:
    KomoDecTest1Obstacle()
    {
        n_obstacles = 1;
    }
};

class KomoDecTest2Obstacles : public KomoDecTest
{
  public:
    KomoDecTest2Obstacles()
    {
        n_obstacles = 2;
    }
};

class KomoDecTest3Obstacles : public KomoDecTest
{
  public:
    KomoDecTest3Obstacles()
    {
        n_obstacles = 3;
    }
};


class KomoDecTest4Obstacles : public KomoDecTest
{
  public:
    KomoDecTest4Obstacles()
    {
        n_obstacles = 4;
    }
};

class KomoDecTest5Obstacles : public KomoDecTest
{
  public:
    KomoDecTest5Obstacles()
    {
        n_obstacles = 5;
    }
};

class KomoDecTestLinear1Obstacle : public KomoDecTest
{
   public:
    KomoDecTestLinear1Obstacle()
    {
        n_obstacles = 1;
        tree = false;
    }
};

class KomoDecTestLinear2Obstacle : public KomoDecTest
{
   public:
    KomoDecTestLinear2Obstacle()
    {
        n_obstacles = 2;
        tree = false;
    }
};
