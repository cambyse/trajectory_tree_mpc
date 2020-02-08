#pragma once

#include <control_tree/core/behavior_base.h>

#include <math.h>
#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include <visualization_msgs/MarkerArray.h>

#include <control_tree/qp/MPC_model.h>
#include <control_tree/qp/QP_problem.h>


class StopLineQPLinear : public BehaviorBase
{
public:
    StopLineQPLinear(BehaviorManager&, int steps_per_phase);

    void desired_speed_callback(const std_msgs::Float32::ConstPtr& msg);

    void stopline_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);

    TimeCostPair plan();

    std::vector<nav_msgs::Path> get_trajectories();

private:
    // params
    const uint steps_;
    double u_min_{-8.0};
    double u_max_{2.0};

    MPC_model model_;
    QP_problem solver_;

    // target: params than can be adapted
    double v_desired_;
    double existence_probability_;
    arr stop_position_;

    Vector2d x0_;
    VectorXd U_sol_;
    VectorXd X_sol_;

    // state;
    bool optimization_run_;
};

VectorXd emergency_brake(double v, int n_phases, int steps_per_phase, double u);



