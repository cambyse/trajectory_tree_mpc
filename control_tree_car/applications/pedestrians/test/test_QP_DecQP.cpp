//#include <control_tree/qp/QP_tree_problem_DecQP.h>

#include <chrono>

#include <qp/control_tree_plot.h>
#include <qp/QP_tree_problem_DecQP.h>
#include "common/qp_problems.h"

#include <common/control_tree.h>

#include <gtest/gtest.h>

TEST_F(QPTest, JOINT_test_2_branches_4_steps_constrained)
{
    auto pb = create_2_branches_4_steps_constrained();

    plan_JointQP(pb, false);
}

TEST_F(QPTest, JOINT_test_4_branches_4_steps_constrained)
{
    auto pb = create_paper_4_branches_4_steps_constrained();

    plan_JointQP(pb, false);
}


TEST_F(QPTest, JOINT_test_10_branches_4_steps_constrained)
{
    auto pb = create_N_branches_4_steps_constrained(30);

    plan_JointQP(pb, false);
}

TEST_F(QPTest, JOINT_test_paper_1_branch_4_steps)
{
    auto pb = create_paper_1_branch_4_steps_constrained();

    vel_axis.range = "[0:15]";
    acc_axis.range = "[-8:3]";

    auto U = plan_JointQP(pb, false, "/tmp/1_branch.dat");//, "/tmp/1_branch.dat");

    //auto U = plan_JointQP(pb, false, "/home/camille/Phd/Paper/T-RO-2024/plots/1_branch.dat");//, "/tmp/1_branch.dat");
}

TEST_F(QPTest, DEC_paper_4_branches_4_steps_constrained)
{
    auto pb = create_paper_4_branches_4_steps_constrained(0.1);

    vel_axis.range = "[0:15]";
    acc_axis.range = "[-8:3]";

    plan_DecQP(pb, false, "/tmp/4_branches.dat");

    //plan_DecQP(pb, false, "/home/camille/Phd/Paper/T-RO-2024/plots/4_branches.dat");
}

TEST_F(QPTest, DEC_test_paper_1_branch_4_steps)
{
    auto pb = create_paper_1_branch_4_steps_constrained();

    vel_axis.range = "[0:15]";
    acc_axis.range = "[-8:3]";

    plan_DecQP(pb, false);//, "/tmp/1_branch.dat");
    // plan_DecQP(pb, false);//, "/home/camille/Phd/Paper/T-RO-2024/plots/1_branch.dat");
}

TEST_F(QPTest, DEC_test_paper_multi_probabilities)
{
    std::vector<double> ps{0.025, 0.05, 0.1, 0.25, 0.5, 0.75, 1.0};

    for(const auto p: ps)
    {
        auto pb = create_paper_4_branches_4_steps_constrained(p);

        vel_axis.range = "[0:15]";
        acc_axis.range = "[-8:3]";

        std::stringstream ss;
        //ss << "/home/camille/Phd/Paper/T-RO-2024/plots/4_branches_" << p << ".dat";
        ss << "/tmp/4_branches_" << p << ".dat";

        plan_DecQP(pb, false, ss.str());
    }
}

TEST_F(QPTest, DEC_test_2_branches_4_steps_constrained)
{
  auto pb = create_2_branches_4_steps_constrained(0.1);

  plan_DecQP(pb, false);
}

TEST_F(QPTest, test_3_branches_4_steps_constrained)
{
    auto pb = create_3_branches_4_steps_constrained();

    plan_DecQP(pb, false);
}

TEST_F(QPTest, test_4_branches_4_steps_constrained)
{
    auto pb = create_4_branches_4_steps_constrained();

    plan_DecQP(pb, false);
}

TEST_F(QPTest, test_replicate_simulation_1)
{
    auto pb = replicate_simulation_1();

    plan_DecQP(pb, true);
}

// runtime optimization!
TEST_F(QPTest, test_5_branches_one_close_obstacle)
{
    auto pb = create_5_branches_one_close_obstacle();

    plan_DecQP(pb, true);
}

TEST_F(QPTest, test_5_branches_two_obstacles)
{
    auto pb = create_5_branches_two_obstacles();

    plan_DecQP(pb, true);
}

TEST_F(QPTest, test_5_branches_two_unlikely_obstacles)
{
    auto pb = create_5_branches_two_unlikely_obstacles();

    plan_DecQP(pb, true);
}

TEST_F(QPTest, test_benchmark_5_branches)
{
  //file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;

  auto pb_1 = create_5_branches_one_close_obstacle();
  auto pb_2 = create_5_branches_two_obstacles();
  auto pb_3 = create_5_branches_two_unlikely_obstacles();

  std::vector<double> stopTols{0.1, 0.05, 0.01};
  std::vector<double> aulaMuInits{1.0, 1.2, 1.5, 2.0};
  //std::vector<double> aulaMuIncs{1.0, 1.2, 1.5, 2.0};
  std::vector<double> admmMuInits{1.0, 2.0, 5.0, 10.0};
  //std::vector<double> admmMuIncs{1.0, 1.2, 1.5, 2.0};
  std::vector<double> muIncs{1.0, 1.2, 1.5, 2.0};

  for(const auto& stopTol: stopTols)
  {
    std::stringstream ss; ss << "/tmp/benchmark_" <<  stopTol << ".dat";
    std::ofstream file(ss.str());

    for(const auto& aulaMuInit: aulaMuInits)
    {
      for(const auto& muInc: muIncs)
      {
//      for(const auto& aulaMuInc: aulaMuIncs)
//      {
        for(const auto& admmMuInit: admmMuInits)
        {
//          for(const auto& admmMuInc: admmMuIncs)
//          {
            BenchmarkParams params;
            params.aulaMuInit = aulaMuInit;
            params.aulaMuInc = muInc;
            params.admmMuInit = admmMuInit;
            params.admmMuInc = muInc;
            params.stopTol = stopTol;

            plan_DecQP(pb_1, false,  "", PARALLEL, params);

            //file << aulaMuInit << " " << aulaMuInc << " " << admmMuInit << " " << admmMuInc << " " << evals << " " << execution_time_ms << std::endl;

            auto evals_1 = evals;
            evals = 0;

            plan_DecQP(pb_2, false,  "", PARALLEL, params);

            //file << aulaMuInit << " " << aulaMuInc << " " << admmMuInit << " " << admmMuInc << " " << evals << " " << execution_time_ms << std::endl;

            auto evals_2 = evals;
            evals = 0;

            plan_DecQP(pb_3, false,  "", PARALLEL, params);

            //file << aulaMuInit << " " << aulaMuInc << " " << admmMuInit << " " << admmMuInc << " " << evals << " " << execution_time_ms << std::endl;

            auto evals_3 = evals;
            evals = 0;

            double eval_l2_average = sqrt(evals_1 * evals_1 + evals_2 * evals_2 + evals_3 * evals_3);

            file << params.aulaMuInit << " " << params.aulaMuInc << " " << params.admmMuInit << " " << params.admmMuInc << " " << evals_1 << " " << evals_2 << " " << evals_3 << " " <<  eval_l2_average<< std::endl;
          }
        }
      //}
    }
  }
}

//

TEST_F(QPTest, test_5_branches_four_obstacles_first_is_certain)
{
    auto pb = create_5_branches_four_obstacles_first_is_certain();

    plan_DecQP(pb, true);
}

TEST_F(QPTest, test_20_branches_)
{
    std::ofstream file("/tmp/execution_time_dec_qp_20.dat");
    //std::ofstream file("/home/camille/Phd/Paper/T-RO-2024/plots/execution_time_dec_qp_20.dat");

    file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
    for(auto i = 1; i <= 20; ++i)
    {
        auto pb = create_N_branches_4_steps_constrained(i);
        plan_DecQP(pb, false, "", THREAD_POOL);
        file << "  " << i << " " << execution_time_ms << std::endl;
    }
}

TEST_F(QPTest, test_20_branches_joint) //-> interesting test, really shows the benefit of decomposing!
{
    std::ofstream file("/tmp/execution_time_joint_dec_qp_20.dat");

    file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
    for(auto i = 1; i <= 20; ++i)
    {
        auto pb = create_N_branches_4_steps_constrained(i);
        plan_JointQP(pb, false);
        file << "  " << i << " " << execution_time_ms << std::endl;
    }
}

TEST_F(QPTest, test_N_branches_dec)
{
    //std::ofstream file("/home/camille/Phd/Paper/ICRA-2021/plots/execution_time_dec_qp_100_.dat");
    std::ofstream file("/tmp/execution_time_dec_qp_100_.dat");

    file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
    for(auto i = 1; i <= 100; ++i)
    {
        auto pb = create_N_branches_4_steps_constrained(i);
        plan_DecQP(pb, false, {}, PARALLEL);
        file << "  " << i << " " << execution_time_ms << std::endl;
    }
}

TEST_F(QPTest, test_dec_beyond_100_branches)
{
  std::ofstream file("/tmp/execution_time_dec_qp_beyond_100_.dat");

  file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
  for (auto i: {100, 200, 300, 400, 500, 600, 700})
  {
    auto pb = create_N_branches_4_steps_constrained(i);
    plan_DecQP(pb, false, {}, PARALLEL);
    file << "  " << i << " " << execution_time_ms << std::endl;
  }
}

TEST_F(QPTest, test_N_branches_joint)
{
    //std::ofstream file("/home/camille/Phd/Paper/ICRA-2021/plots/execution_time_dec_qp_100_.dat");
    std::ofstream file("/tmp/execution_time_joint_qp_100_.dat");

    file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
    for(auto i = 1; i <= 100; ++i)
    {
        auto pb = create_N_branches_4_steps_constrained(i);
        plan_JointQP(pb, false);
        file << "  " << i << " " << execution_time_ms << std::endl;
    }
}

TEST_F(QPTest, test_4_branches_4_steps_constrained_with_thread_pool)
{
    auto pb = create_4_branches_4_steps_constrained();

    plan_DecQP(pb, false, "", THREAD_POOL);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

