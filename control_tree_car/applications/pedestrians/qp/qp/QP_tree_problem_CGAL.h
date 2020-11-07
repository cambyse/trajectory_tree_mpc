#pragma once

#include <qp/MPC_model.h>
#include <qp/QP_constraints.h>
#include <qp/QP_tree_solver_base.h>

#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

typedef double ET;

// program and solution types
typedef CGAL::Quadratic_program<ET> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

class QP_tree_problem_CGAL : public QP_tree_joint_solver_base
{
public:
    QP_tree_problem_CGAL(const MPC_model & mpc,
                    double u_min, double u_max);

private:
    VectorXd call_solver() override;

private:
    Program qp;
};