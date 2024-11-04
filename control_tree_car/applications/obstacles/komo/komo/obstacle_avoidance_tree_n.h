#pragma once

#include <common/behavior_base.h>

#include <math.h>
#include <boost/bind.hpp>
#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <KOMO/komo.h>

#include <tree_builder.h>
#include <car_kinematic.h>
#include <velocity.h>
#include <axis_bound.h>
#include <circular_obstacle.h>
#include <komo/komo_factory.h>

#include <visualization_msgs/MarkerArray.h>

#include <Optimization/decentralized_optimizer.h>
#include <Optimization/decentralized_lagrangian.h>

// Use for unit testing only and timing!
class ObstacleAvoidanceTreeN : public BehaviorBase
{
public:
    ObstacleAvoidanceTreeN(BehaviorManager&, const std::string& kin_path, int n_obstacles, int steps_per_phase);

    void desired_speed_callback(const std_msgs::Float32::ConstPtr& msg);

    void obstacle_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);

    TimeCostPair plan();

    std::vector<nav_msgs::Path> get_trajectories();

    void set_optim_callback(const std::function<void()>& _) {}

    static uint n_branches(uint n_obstacles, bool tree) { return tree ? pow(2.0, n_obstacles) : 1; }

private:
    void init_tree();
    void update_groundings();

private:
    // params
    const uint n_obstacles_; // number of obstacles
    const uint n_branches_; // number of branches

    rai::KinematicWorld kin_;
    const uint horizon_; // planning horizon number of phases ~ [s]
    const uint steps_;

    // target: params than can be adapted
    double v_desired_;
    std::vector<Obstacle> obstacles_;

    // komo
    KomoFactory komo_factory_;

    // functional
    std::vector<Objectives> objectivess_;

    // state;
    mp::TreeBuilder komo_tree_;

    std::vector<double> belief_state_;

    std::vector<intA> varss_branch_order_0_;
    std::vector<intA> varss_branch_order_1_;
    std::vector<intA> varss_branch_order_2_;
    intA vars_all_order_1_;

    std::shared_ptr<KOMO> komo_;
    std::vector<std::shared_ptr<KOMO::Conv_MotionProblem_GraphProblem>> converters_;
    std::vector<std::shared_ptr<ConstrainedProblem>> constrained_problems_;

    arr x_;
    std::vector<arr> xmasks_;

    DecOptConfig options_;
};


