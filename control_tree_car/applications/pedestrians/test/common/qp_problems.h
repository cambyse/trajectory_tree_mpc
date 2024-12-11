#pragma once

#include <common/control_tree.h>
#include <qp/MPC_model.h>
#include <qp/QP_constraints.h>
#include <qp/QP_tree_solver_base.h>
#include <qp/control_tree_plot.h>
#include <Optimization/qp_lagrangian.h>
#include <boost/optional.hpp>
#include <gtest/gtest.h>

struct QP_problem
{
    MPC_model model;
    TreePb tree;
    Constraints k;
    Vector2d x0;
    Vector2d xd;
};

QP_problem create_2_branches_2_steps_constrained(double p=0.6);
QP_problem create_2_branches_4_steps_constrained(double p=0.6);
QP_problem create_3_branches_4_steps_constrained(double p=0.6);
QP_problem create_4_branches_4_steps_constrained(double p=0.6);
QP_problem create_10_branches_4_steps_constrained();
QP_problem create_20_branches_4_steps_constrained();
QP_problem create_N_branches_4_steps_constrained(int N);
QP_problem create_2_stages_branching(double p=0.6);
// paper plots
QP_problem create_paper_1_branch_4_steps_constrained(double p=0.6);
QP_problem create_paper_4_branches_4_steps_constrained(double p=0.6);
QP_problem replicate_simulation_1();

// runtime optimization - benchmark to find good params
QP_problem create_5_branches_one_close_obstacle();
QP_problem create_5_branches_two_obstacles();
QP_problem create_5_branches_two_unlikely_obstacles();
QP_problem create_5_branches_four_obstacles_first_is_certain();

struct BenchmarkParams
{
  double aulaMuInit{1.0};
  double aulaMuInc{2.0};
  double admmMuInit{1.0};
  double admmMuInc{1.0};

  double stopTol{0.01};
};

class QPTest : public ::testing::Test
{
public:
    double execution_time_ms{0};
    uint evals{0}; // number of iterations of the subproblem which was requiring the most of them

 protected:
    VectorXd plan_OSQP(const QP_problem &pb, bool plot = false, const std::string & filename = "");
    VectorXd plan_JointQP(const QP_problem &pb, bool plot = false, const std::string & filename = "");
    VectorXd plan_DecQP(const QP_problem &pb, bool plot = false, const std::string & filename = "", Mode scheduling = PARALLEL, boost::optional<BenchmarkParams> = {});

    void plot_XU(const VectorXd& X, const VectorXd& U, const QP_problem &pb) const;
    void save_XU(const VectorXd& X, const VectorXd& U, const QP_problem &pb, const std::string & filename) const;

    PlotAxis acc_axis{"acceleration", "[-4:3]"};
    PlotAxis vel_axis{"velocity", "[0:10]"};
    PlotAxis x_axis{"x", "[0:50]"};

    double u_min{-6.0};
    double u_max{2.0};
};

