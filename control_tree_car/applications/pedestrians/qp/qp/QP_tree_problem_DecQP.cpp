#include <qp/QP_tree_problem_DecQP.h>
#include <qp/QP_tree_problem_DecQP_utils.h>

//----Joint------------------------//

QP_tree_problem_JointQP::QP_tree_problem_JointQP(const MPC_model & mpc, double u_min, double u_max, CallBackType run_start_callback, StepCallBackType run_end_callback, StepCallBackType step_callback)
    : QP_tree_joint_solver_base(mpc, u_min, u_max)
    , options(PARALLEL, false, NOOPT, false, step_callback, run_start_callback, run_end_callback)
{
  options.opt.verbose = 0;
  //options.opt.aulaMuInc = 1;
}

VectorXd QP_tree_problem_JointQP::call_solver()
{
    const auto P = convert(H);
    const auto q = convert(C);
    const auto K = createK(KA);
    const auto u = createU(Up, Lo);

    auto qp = std::make_shared<QP_Problem>(P, q, K, u);

    std::vector<std::shared_ptr<QP_Problem>> pbs;
    pbs.push_back(qp);

    arr x = zeros(P.d0);

    DecOptConstrained<QP_Problem, AverageUpdater> opt(x, pbs, {}, AverageUpdater(), options);

    opt.run();

    return convert(x);
}

//----Dec---------------------------//

QP_tree_problem_DecQP::QP_tree_problem_DecQP(const MPC_model & mpc, double u_min, double u_max, Mode scheduling, CallBackType run_start_callback, StepCallBackType run_end_callback, StepCallBackType step_callback)
  : QP_tree_solver_base(mpc, u_min, u_max)
  , options(scheduling, true, NOOPT, false, step_callback, run_start_callback, run_end_callback)
{
  options.opt.verbose = 0;
  // options.opt.aulaMuInc = 1.0; -> no effect!

  // print options!
//  Mode scheduling;

//  bool compressed; // wether xs.d0 == x.d0, if true subproblem optimizers act on smaller (local) x
//  OptOptions opt;  // for newton and aula

//  bool checkGradients;
//  CallBackType callback; // called after each step() (for debugging)
//  CallBackType run_start_callback; // called when starting the run method (for timing)
//  CallBackType run_end_callback;   // called when ending the run method (for timing)
//  ostream *logFile;

//  double muInit{1.0}; // initial mu after first step
//  double muInc{1.0};  // mu increase

//  const double muInc = 1.2;

//  options.muInc = 1.2;
//  options.muInit = 10.0; // initial mu after first step
//  options.opt.aulaMuInc = 2.0;
//  options.opt.stopTolerance = 0.05;

  // those values were obtained via grid search, look for test_benchmark_5_branches
  options.opt.muInit = 1.0; // initial mu after first step
  options.opt.aulaMuInc = 1.5;
  options.muInit = 1.0;     // initial mu after first step
  options.muInc = 1.5;

//  options.opt.stopTolerance = 0.05;

//  std::cout << "u_max:" << u_max << std::endl;
//  std::cout << "u_min:" << u_min << std::endl;
//  std::cout << "scheduling:" << options.scheduling << std::endl;
//  std::cout << "compressed:" << options.compressed << std::endl;
//  std::cout << "opt:" << options.opt << std::endl;
//  std::cout << "stopTolerance:" << options.opt.stopTolerance << std::endl;
//  std::cout << "checkGradients:" << options.checkGradients << std::endl;
//  std::cout << "muInit:" << options.muInit << std::endl;
//  std::cout << "muInc:" << options.muInc << std::endl;
}

VectorXd QP_tree_problem_DecQP::solve(const Vector2d & x0, const Vector2d & xd, const Constraints& joint_k,
                                      int n_steps,
                                      const std::vector<IntA> & joint_varss,
                                      const std::vector<Arr> & joint_scaless)
{
  // generate compressed var and masks
  IntA var, global_to_branch;
  const auto masks = get_compressed_masks(n_steps, mpc.get_dim(), joint_varss, var, global_to_branch);
  const auto bs = get_belief_state(joint_scaless);
//  const auto scales = get_one_scale(joint_scaless);
  const auto scaless = get_compressed_scales(joint_scaless);

  // compress constraints
  const auto ks = get_compressed_constraints(joint_k, var, global_to_branch);

  // build subproblems
  const auto branch_n_steps = var.size();
  std::vector<IntA> branch_varss({var});
  std::vector<std::shared_ptr<QP_Problem>> pbs; pbs.reserve(masks.size());

  std::vector<std::future<std::shared_ptr<QP_Problem>>> futures;

  futures.reserve(masks.size());
  for(uint i = 0; i < masks.size(); ++i)
  {
    futures.push_back(
          std::async(std::launch::async,
                     [&, i]()
                      {
                        return build_qp(i, branch_n_steps, branch_varss, {scaless[i]}/*{scales}*/, ks, x0, xd);
                      }
                    )
    );
  }

  for(auto& future: futures)
  {
    pbs.emplace_back(future.get());
  }

  // solve
  arr x = zeros(n_steps * mpc.get_dim());

  //DecOptConstrained<QP_Problem, BeliefState> opt(x, pbs, masks, BeliefState(bs), options);
  DecOptConstrained<QP_Problem, AverageUpdater> opt(x, pbs, masks, AverageUpdater(), options);

  opt.run();

  return convert(x);
}

std::shared_ptr<QP_Problem> QP_tree_problem_DecQP::build_qp(int i,
                                                            int n_steps,
                                                            const std::vector<IntA>& varss,
                                                            const std::vector<Arr>& scaless,
                                                            const std::unordered_map<int, Constraints> & constraints,
                                                            const Vector2d & x0,
                                                            const Vector2d & xd) const
{
  // build MPC matrices
  // costs (could be computed just once and scaled?)
  const auto S = mpc.get_S(n_steps, varss);
  const auto T = mpc.get_T(n_steps, varss);

  const auto Q_bar = mpc.get_Q_bar(n_steps, varss, scaless);
  const auto R_bar = mpc.get_R_bar(n_steps, varss, scaless);

  const auto H = 2 * (R_bar + S.transpose() * Q_bar * S);
  const auto F = 2 * (T.transpose() * Q_bar * S);
  const auto G = 2 * Q_bar * S;

  const Eigen::VectorXd & Xd = mpc.get_Xd(xd, n_steps);
  const auto C = (x0.transpose() * F).transpose() - (Xd.transpose() * G).transpose();

  // constraints
  Eigen::MatrixXd KA;
  Eigen::VectorXd Up, Lo;

  const auto kit = constraints.find(i);
  if(kit!=constraints.end())
  {
    const auto& k = kit->second;
    const Eigen::VectorXd Xmax = k.getXmax();
    const Eigen::MatrixXd Sextract = k.getSextract();

    const auto nk = Sextract.rows() + H.rows();
    KA.resize(nk, H.rows());
    Up.resize(nk);
    Lo.resize(nk);

    // traj constraints
    KA.block(0, 0, Sextract.rows(), H.rows()) = Sextract * S;
    Up.head(Sextract.rows()) = Sextract * (Xmax - T * x0);
    Lo.head(Up.rows()) = VectorXd::Constant(Up.rows(), std::numeric_limits<double>::lowest());

    // control bounds constraints
    KA.block(Sextract.rows(), 0, H.rows(), H.rows()) = MatrixXd::Identity(H.rows(), H.rows());
    Up.tail(H.rows()) = VectorXd::Constant(H.rows(), u_max_);
    Lo.tail(H.rows()) = VectorXd::Constant(H.rows(), u_min_);
  }
  else // no traj constraints, add control bounds only
  {
    // control bounds constraints
    KA = MatrixXd::Identity(H.rows(), H.rows());
    Up = VectorXd::Constant(H.rows(), u_max_);
    Lo = VectorXd::Constant(H.rows(), u_min_);
  }

  // build QP matrices
  const auto P = convert(H);
  const auto q = convert(C);
  const auto K = createK(KA);
  const auto u = createU(Up, Lo);

  return std::make_shared<QP_Problem>(P, q, K, u);
}
