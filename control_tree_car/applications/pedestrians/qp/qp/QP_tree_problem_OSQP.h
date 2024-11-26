#pragma once

#include <qp/MPC_model.h>
#include <qp/QP_constraints.h>
#include <qp/QP_tree_solver_base.h>

#include "osqp.h"

using CallBackType = std::function<void()>;

csc * create_csc_matrix(const MatrixXd & M);

class QP_tree_problem_OSQP : public QP_tree_joint_solver_base
{
public:
    QP_tree_problem_OSQP(const MPC_model & mpc,
                         double u_min, double u_max,
                         CallBackType run_start_callback = {}, CallBackType run_end_callback = {});

    ~QP_tree_problem_OSQP();

private:
    VectorXd call_solver() override;

private:
    OSQPWorkspace *work;
    OSQPSettings  *settings;
    OSQPData      *data;

    CallBackType run_start_callback;
    CallBackType run_end_callback;
};
