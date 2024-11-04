#pragma once

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/Odometry.h>

#include <Kin/kin.h>

#include <common/utility.h>
#include <unordered_map>
#include <tree_builder.h>
#include <circular_obstacle.h>


class Objective;
class KOMO;

struct Costs
{
    double total;
    std::unordered_map<std::string, double> costs;
};

geometry_msgs::PoseStamped kin_to_pose_msg(const rai::KinematicWorld * kin);
geometry_msgs::PoseStamped kin_1d_to_pose_msg(const rai::KinematicWorld * kin);
Costs traj_cost(const WorldL & Gs, const std::list<Objective *> & objectives);
WorldL get_traj_start(const WorldL & configurations, int start = 0, int end = 2);
void unify_prefix(std::vector<std::shared_ptr<KOMO>>& komos);
void init_komo_with_constant_velocity_trajectory(const std::shared_ptr<KOMO> & komo, const OdometryState & o, uint steps, const intA& vars_all_order_1);
void shift_komos(std::vector<std::shared_ptr<KOMO>>& komos, const OdometryState & o, uint steps);
//inline int shift_komos(const std::shared_ptr<KOMO>& komo, const OdometryState & o, uint steps) {NIY;return 0;}
//void shift_dual(DualState& state, int dual_dim_per_step, int index);
//void shift(arr& state, uint n);
std::vector<Pose2D> convert(const std::shared_ptr<KOMO>& komo);
void slide_trajectory(uint index, uint steps, std::vector<Pose2D>& trajectory);
void slide_trajectory(uint index, double mu, uint steps, std::vector<Pose2D>& trajectory);
Pose2D interpolate(const Pose2D& a, const Pose2D& b, double mu);
void translate_trajectory(const OdometryState & o, uint steps, std::vector<Pose2D>& trajectory);
void first_guess_trajectory(const OdometryState & o, uint steps, std::vector<Pose2D>& trajectory);
void first_guess_trajectory(const OdometryState & o, double v_min, uint steps, std::vector<Pose2D>& trajectory);
void update_komo(const std::vector<Pose2D>& trajectory, const std::shared_ptr<KOMO>& komo);
void update_x(arr&x, const std::vector<std::shared_ptr<KOMO>>& komos, const std::vector<intA>& vars);
void update_x(arr&x, const std::shared_ptr<KOMO>& komo);

void convert(uint n_branches, uint horizon, mp::TreeBuilder& tb);
std::vector<double> fuse_probabilities(const std::vector<Obstacle>&, bool tree, std::vector<std::vector<bool>> &);
void convert(uint n_branches, uint horizon, mp::TreeBuilder& komo_tree_);
std::vector<Obstacle> get_relevant_obstacles(const std::vector<Obstacle>& obstacles, const std::vector<bool>& activities);

template<bool tree>
inline std::vector<double> fuse(const std::vector<Obstacle>& obstacles, std::vector<std::vector<bool>> & activities)
{

}

template<>
inline std::vector<double> fuse<true>(const std::vector<Obstacle>& obstacles, std::vector<std::vector<bool>> & activities)
{
    const uint n = pow(2.0, obstacles.size());

    std::vector<double> probabilities(n, 0.0);
    activities = std::vector<std::vector<bool>>(n);

    // compute activities
    for(auto i = 0; i < obstacles.size(); ++i)
    {
      bool active = true;
      uint rythm = pow(2.0, double(i));
      for(auto j = 0; j < n; ++j)
      {
        if(j > 0 && j % rythm == 0)
        {
          active = !active;
        }
        activities[j].push_back(active);
      }
    }

    // fuse
    for(auto j = 0; j < n; ++j)
    {
      auto p = 1.0;
      for(auto i = 0; i < obstacles.size(); ++i)
      {
        if(activities[j][i])
          p *= obstacles[i].p;
        else
          p *= (1.0 - obstacles[i].p);
      }

      probabilities[j] = p;
    }

    return probabilities;
}

template<>
inline std::vector<double> fuse<false>(const std::vector<Obstacle>& obstacles, std::vector<std::vector<bool>> & activities)
{
    const uint n = 1;

    std::vector<double> probabilities(n, 0.0);
    activities = std::vector<std::vector<bool>>(n);

    // compute activities
    for(auto i = 0; i < obstacles.size(); ++i)
    {
      activities[0].push_back(true);
    }

    // fuse
    probabilities[0] = 1.0;

    return probabilities;
}
