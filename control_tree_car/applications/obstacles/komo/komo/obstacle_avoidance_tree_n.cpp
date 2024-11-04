#include <komo/obstacle_avoidance_tree_n.h>

#include <common/behavior_manager.h>

#include <komo/velocity_axis.h>

#include <common/utility.h>
#include <komo/utility_komo.h>


ObstacleAvoidanceTreeN::ObstacleAvoidanceTreeN(BehaviorManager& behavior_manager, const std::string& kin_path, int n_obstacles, int steps_per_phase)
    : BehaviorBase(behavior_manager)
    , n_obstacles_(n_obstacles)
    , n_branches_(n_branches(n_obstacles, true))
    , kin_(kin_path.c_str()) //(ros::package::getPath("control_tree_car") + "/data/LGP-real-time.g").c_str())
    , steps_(steps_per_phase)
    , horizon_(5)
    , v_desired_(1.0)
    , obstacles_(n_obstacles_, {arr{-10, 0, 0}, 0.0})
    , komo_factory_(kin_path)
    , komo_tree_(1.0, 0)
    , options_(PARALLEL, true, NOOPT, false)
{
    options_.opt.verbose = 0;
    options_.opt.aulaMuInc = 1;
    options_.muInit = 2.0;
    options_.muInc = 2.0;

    // optim structure
    init_tree();

    // komo
    komo_ = std::make_shared<KOMO>();
    komo_->sparseOptimization = true;
    komo_->setModel( kin_, false );
    komo_->setTiming(komo_tree_.n_nodes() - 1, steps_, 1);
    komo_->verbose = 0;

    // komo
    std::vector<std::vector<bool>> activities;
    fuse_probabilities(obstacles_, true, activities); // branch probabilities

    for(auto i = 0; i < n_branches_; ++i)
    {
      // objectives
      const auto obstacles = get_relevant_obstacles(obstacles_, activities[i]);

      Objectives objectives = komo_factory_.ground_komo(komo_, obstacles, 0.0, v_desired_);
      objectives.acc_->vars = varss_branch_order_2_[i];
      objectives.ax_->vars = varss_branch_order_0_[i];
      objectives.vel_->vars = varss_branch_order_1_[i];
      objectives.car_kin_->vars = varss_branch_order_1_[i];

      if(!obstacles.empty())
      {
        objectives.collision_avoidance_->vars = varss_branch_order_0_[i];
      }

      objectivess_.push_back(objectives);
    }

    komo_->reset(0);

    // opt pb
    auto gp = std::make_shared<KOMO::Conv_MotionProblem_GraphProblem>(*komo_);
    auto pb = std::make_shared<Conv_Graph_ConstrainedProblem>(*gp, komo_->logFile);

    converters_.push_back(gp);
    constrained_problems_.push_back(pb);

    x_ = komo_->x;
    xmasks_.push_back(ones(komo_->x.d0));

    // debug
//    std::cout << "///" << std::endl;
//    std::cout << vars_branch_1_order_1_ << std::endl;
//    std::cout << "---" << std::endl;
//    std::cout << vars_branch_2_order_1_ << std::endl;
//    std::cout << "---" << std::endl;
//    std::cout << vars_all_order_0_ << std::endl;
//    std::cout << "---" << std::endl;
//    std::cout << vars_all_order_1_ << std::endl;
//    std::cout << "---" << std::endl;
//    std::cout << vars_all_order_2_ << std::endl;
//    std::cout << "---" << std::endl;
//    std::cout << scales_all_ << std::endl;
}

void ObstacleAvoidanceTreeN::desired_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
    //ROS_INFO( "update desired speed.." );

    v_desired_ = msg->data;
}

void ObstacleAvoidanceTreeN::obstacle_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  CHECK_EQ(msg->markers.size(), obstacles_.size() * 2, "number of obstacles are not consistent");

  for(auto i = 0; i < msg->markers.size() / 2; ++i)
  {
    const auto&m = msg->markers[2 * i + 1];

    /// position and geometry
    obstacles_[i].position = {m.pose.position.x - 0.5, m.pose.position.y, 0};
    obstacles_[i].p = m.color.a;
    //obstacles_[i].radius = m.scale.x;
    obstacles_[i].radius = m.scale.x / 2 + 0.5; //-> video only

    //std::cout << "p[" << i << "] =" << obstacles_[i].p << std::endl;
  }
}

TimeCostPair ObstacleAvoidanceTreeN::plan()
{
    update_groundings();

    //ROS_INFO( "ObstacleAvoidanceTree::plan.." );

    if(obstacles_.front().position.d0 == 0)
    {
        ROS_INFO( "ObstacleAvoidanceTree::plan.. abort planning" );

        return {0.0, 0.0};
    }

    // init
    auto o = manager_.odometry();

    init_komo_with_constant_velocity_trajectory(komo_, o, steps_, vars_all_order_1_); // NIY!
    update_x(x_, komo_);
    // reset

    auto start = std::chrono::high_resolution_clock::now();

    komo_->reset();

    DecOptConstrained<ConstrainedProblem, AverageUpdater> opt(x_, constrained_problems_, xmasks_, AverageUpdater(), options_);

    // run
    opt.run();

    //komo_->run();
    //komo_->getReport(true);
    //komo_->plotTrajectory();
    auto end = std::chrono::high_resolution_clock::now();
    float execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    ROS_INFO( "[tree joint n] execution time (ms): %f", execution_time_us / 1000 );

    // evaluate costs
    auto Gs = get_traj_start(komo_->configurations);
    //auto cost = traj_cost(Gs, {acc_, ax_, vel_});
    //

    return {execution_time_us / 1000000, 0.0 /*cost.total*/};
}

std::vector<nav_msgs::Path> ObstacleAvoidanceTreeN::get_trajectories()
{
  std::vector<nav_msgs::Path> trajs;
  trajs.reserve(n_branches_);

  const auto t = ros::Time::now();
  for(auto i = 0; i < n_branches_; ++i)
  {
    nav_msgs::Path msg;
    msg.header.stamp = t;
    msg.header.frame_id = "map";

    msg.poses.push_back(kin_to_pose_msg(komo_->configurations(0)));
    msg.poses.push_back(kin_to_pose_msg(komo_->configurations(1)));

    for(const auto k: varss_branch_order_0_[i])
    {
      const auto& kin = komo_->configurations(k+2); // order 2
      msg.poses.push_back(kin_to_pose_msg(kin));
    }

    trajs.push_back(msg);
  }

  return trajs;
}

void ObstacleAvoidanceTreeN::init_tree()
{
  convert(n_branches_, horizon_, komo_tree_);

  for(std::size_t i = 0; i < n_branches_; ++i)
  {
    auto vars_branch_1_order_0_ = komo_tree_.get_vars({0.0, horizon_}, komo_tree_.get_leaves()[i], 0, steps_);
    auto vars_branch_1_order_1_ = komo_tree_.get_vars({0.0, horizon_}, komo_tree_.get_leaves()[i], 1, steps_);
    auto vars_branch_1_order_2_ = komo_tree_.get_vars({0.0, horizon_}, komo_tree_.get_leaves()[i], 2, steps_);

    varss_branch_order_0_.push_back(vars_branch_1_order_0_);
    varss_branch_order_1_.push_back(vars_branch_1_order_1_);
    varss_branch_order_2_.push_back(vars_branch_1_order_2_);

    vars_all_order_1_.append(vars_branch_1_order_1_);
  }
}

void ObstacleAvoidanceTreeN::update_groundings()
{
  std::vector<std::vector<bool>> activities;
  belief_state_ = fuse_probabilities(obstacles_, true, activities); // branch probabilities

  for(auto i = 0; i < n_branches_; ++i)
  {
    auto& objectives = objectivess_[i];

    // update scaling based on belief state
    const auto scales = belief_state_[i] * ones(objectives.acc_->vars.d0);
    objectives.acc_->scales = scales;
    objectives.ax_->scales = scales;
    objectives.vel_->scales = scales;

//    std::cout << objectives.acc_->scales << std::endl;
//    std::cout << objectives.acc_->vars.d0 << std::endl;

//    for(const auto& p: belief_state_)
//    {
//      std::cout << p << ", ";
//    }
//    std::cout << std::endl;

    // set target
    objectives.vel_->map->target = {v_desired_};

    // update collision avoidance
    auto obstacles = get_relevant_obstacles(obstacles_, activities[i]);

    if(!obstacles.empty())
    {
      objectives.circular_obstacle_->set_obstacles(obstacles);
    }
  }

  // TODO: update scales!
}

