#include <control_tree/core/behavior_manager.h>
#include <control_tree/core/utility.h>

#include <control_tree/qp/stopline_qp_linear.h>

StopLineQPLinear::StopLineQPLinear(BehaviorManager& behavior_manager, int steps_per_phase)
    : BehaviorBase(behavior_manager)
    , steps_(steps_per_phase)
    , model_(1.0 / steps_per_phase, 1.0, 5.0)
    , solver_(model_, 4 * steps_)
    , v_desired_(48/3.6)
    , existence_probability_(0.5)
    , optimization_run_(false)
{    

}


void StopLineQPLinear::desired_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
    //ROS_INFO( "update desired speed.." );

    v_desired_ = msg->data;
}

void StopLineQPLinear::stopline_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    const auto N = msg->markers.size() / 3;
    const auto & o = manager_.odometry();

    stop_position_ = {std::numeric_limits<double>::infinity()};
    existence_probability_ = 0.0;

    //ROS_INFO_STREAM( "position " << o.x);

    for(auto i = 0; i < N; ++i)
    {
        /// position and geometry
        auto stop = msg->markers[3*i].pose.position.x - 4.3 + 1.5 - 2.5; // saftey distance
        /// existence probability
        auto existence_probability = msg->markers[3*i+1].color.a;
        if(stop < stop_position_[0] && o.x < stop + 1 && existence_probability > 0.01)
        {
            stop_position_ = {stop};
            existence_probability_ = existence_probability;
        }

        //ROS_INFO_STREAM( "stopline " << i << " " <<  stop << " " << existence_probability);
    }

    //ROS_INFO_STREAM( "stopline callback.." << N << " " << stop_position_(0) );
}

TimeCostPair StopLineQPLinear::plan()
{
    //ROS_INFO( "plan.." );

    const auto & o = manager_.odometry();

    // INITIAL STATES
    x0_ = Vector2d();
    x0_ << o.x, o.v;

    // DESIRED
    Vector2d xd;
    xd << 0, v_desired_;

    // CONSTRAINT
    Vector2d xmax;
    if(stop_position_.size() == 0 || existence_probability_ < 0.01 || o.x > stop_position_[0] + 1)
    {
        xmax << o.x + 1000, 0;
    }
    else
    {
        xmax << stop_position_[0], 0;
    }

    /// Solver
    auto start = std::chrono::high_resolution_clock::now();

    U_sol_ = solver_.solve(x0_, xd, xmax, u_min_, u_max_);
    X_sol_ = model_.predict_trajectory(x0_, U_sol_);

    auto end = std::chrono::high_resolution_clock::now();
    float execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    optimization_run_ = true;

    //check_trajectories();

    //ROS_INFO( "[linear qp] execution time (ms): %f", execution_time_us / 1000 );

    return {execution_time_us / 1000000, model_.cost(x0_, U_sol_[0], xd)};
}

std::vector<nav_msgs::Path> StopLineQPLinear::get_trajectories()
{
    if(!optimization_run_)
    {
        return {nav_msgs::Path(), nav_msgs::Path()};
    }

    nav_msgs::Path msg_1;
    msg_1.header.stamp = ros::Time::now();
    msg_1.header.frame_id = "map";
    msg_1.poses.reserve(X_sol_.rows() / 2 + 1);


    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x0_[0];
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    msg_1.poses.push_back(pose);

    for(auto k = 0; k < int(X_sol_.rows()) / 2; ++k)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = X_sol_[2*k];
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        msg_1.poses.push_back(pose);
    }

    return {msg_1, msg_1};
}

VectorXd emergency_brake(double v, int n_phases, int steps_per_phase, double u)
{
    VectorXd U = VectorXd::Zero(n_phases * steps_per_phase);

    for(auto i = 0; i < U.rows(); ++i)
    {
        double remaining_braking_time = fabs(v / u);

        if(remaining_braking_time > 1.0 / steps_per_phase)
        {
            U[i] = u;
            v += u / steps_per_phase;
        }
        else
        {
            // last step
            U[i] = -v / steps_per_phase;
            v = 0;
            break;
        }
    }

    return U;
}



