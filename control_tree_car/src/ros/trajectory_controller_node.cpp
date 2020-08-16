#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include <ros/package.h>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include <control_tree/core/utility.h>

class TrajectoryController
{
public:
    TrajectoryController(int steps_per_phase)
        : steps_per_phase_(steps_per_phase)
    {

    }

    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // retrieve pose
        odometry_.x = msg->pose.pose.position.x;
        odometry_.y = msg->pose.pose.position.y;
        odometry_.yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);

        // retrieve speeds
        odometry_.v = msg->twist.twist.linear.x;
        odometry_.omega = msg->twist.twist.angular.z;

        odo_received_ = true;
    }

    void trajectory_callback(const nav_msgs::Path::ConstPtr& msg)
    {
        trajectory_ = std::vector<Pose2D>(msg->poses.size());

        for(auto i = 0; i < msg->poses.size(); ++i)
        {
            auto pose = msg->poses[i];
            trajectory_[i] = Pose2D{pose.pose.position.x, pose.pose.position.y, get_yaw_from_quaternion(pose.pose.orientation)};
        }
    }

    std::pair<geometry_msgs::Twist, geometry_msgs::PoseStamped> create_control() const
    {   
        const auto d_nose = 4.3 - 0.9;
        const auto nominal_v = 5; // scale w according to the ratio v / nominal_v

        //// return early if not enough info
        if( trajectory_.size() == 0 || ! odo_received_ )
        {
            return std::pair<geometry_msgs::Twist, geometry_msgs::PoseStamped>();
        }

        // get 0-0
        Pose2D current = {odometry_.x, odometry_.y, odometry_.yaw};

        // get nose
        Pose2D current_nose = {odometry_.x + cos(odometry_.yaw) * d_nose, odometry_.y + sin(odometry_.yaw) * d_nose, odometry_.yaw};

        // project nose on trajectory
        int index = -1;
        int index_nose = -1;
        double mu = -1;

        const auto projected = project_on_trajectory(current, trajectory_, index, mu);
        const auto projected_nose = project_on_trajectory(current_nose, trajectory_, index_nose, mu);

        //ROS_ERROR_STREAM("index:" << index << " size:" << trajectory_.size());
        if(index == -1)
        {
            //ROS_ERROR_STREAM("ego pose doesn't project correctly to planned trajectory");

            //ROS_ERROR_STREAM("current pose:" << current.x << " " << current.y);

//            for(auto p : trajectory_)
//            {
//                ROS_ERROR_STREAM(p.x << " " << p.y);
//            }

            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = 0;
            twist_msg.angular.z = 0;

            return std::pair<geometry_msgs::Twist, geometry_msgs::PoseStamped>(twist_msg, geometry_msgs::PoseStamped());
        }

        /// v
        //double v_index = sqrt(pow(trajectory_[index+1].x - trajectory_[index].x, 2) + pow(trajectory_[index+1].y - trajectory_[index].y, 2)) * steps_per_phase_;
        //double v_index_1 = sqrt(pow(trajectory_[index+2].x - trajectory_[index+1].x, 2) + pow(trajectory_[index+2].y - trajectory_[index+1].y, 2)) * steps_per_phase_;
        //double v = v_index * (1 - mu) + mu * v_index_1;
        int k = index + 1;
        int l = k + 1;
        if(l >= trajectory_.size())
        {
            l = trajectory_.size() - 1;
            k = l - 1;
        }
        double v = sqrt(pow(trajectory_[l].x - trajectory_[k].x, 2) + pow(trajectory_[l].y - trajectory_[k].y, 2)) * steps_per_phase_;
        //double v = sqrt(pow(trajectory_[index+1].x - trajectory_[index].x, 2) + pow(trajectory_[index+1].y - trajectory_[index].y, 2)) * steps_per_phase_;

        /// w
        double w_cmd = 0;
        if(index_nose != -1)
        {
            // distance between corrected nose and current nose
            Eigen::Vector2f u(current_nose.x - projected_nose.x, current_nose.y - projected_nose.y);
            Eigen::Vector2f n(-sin(projected_nose.yaw), cos(projected_nose.yaw));
            auto d = n.dot(u);

            w_cmd = -d;
        }

        w_cmd = w_cmd * (v / nominal_v);

        const double max_w = 1.0;
        if(std::fabs(w_cmd) > max_w)
        {
            //std::cout << "limit omega!" << w_cmd << std::endl;
            //v *= max_w / std::fabs(w_cmd);
            w_cmd = w_cmd > 0 ? max_w : -max_w;
        }

        ///
        tf2::Quaternion q; q.setRPY(0, 0, projected_nose.yaw);
        geometry_msgs::PoseStamped target_pose_msg;
        target_pose_msg.header.stamp = ros::Time::now();
        target_pose_msg.header.frame_id = "map";
        target_pose_msg.pose.position.x = projected_nose.x;
        target_pose_msg.pose.position.y = projected_nose.y;
        target_pose_msg.pose.orientation.x = q.x();
        target_pose_msg.pose.orientation.y = q.y();
        target_pose_msg.pose.orientation.z = q.z();
        target_pose_msg.pose.orientation.w = q.w();

        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = v;
        twist_msg.angular.z = w_cmd;

        return {twist_msg, target_pose_msg};
    }

private:
    // state
    bool odo_received_;
    OdometryState odometry_;
    std::vector<Pose2D> trajectory_;

    // params
    const int steps_per_phase_;
};

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Launch trajectory controller..");

    int steps_per_phase = 1;
    int trajectory_index = 0;

    // ros init
    ros::init(argc, argv, "trajectory_controller");
    ros::NodeHandle n;
    n.getParam("/traj_planner/steps_per_phase", steps_per_phase);
    n.getParam("/traj_controller/trajectory_index", trajectory_index);
    ros::Publisher ctrl_publisher = n.advertise<geometry_msgs::Twist>("/lgp_car/vel_cmd", 1000);
    ros::Publisher target_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/lgp_car/target_pose", 1000);

    TrajectoryController controller(steps_per_phase);

    boost::function<void(const nav_msgs::Path::ConstPtr& msg)> trajectory_callback =
            boost::bind(&TrajectoryController::trajectory_callback, &controller, _1);
    auto trajectory_subscriber = n.subscribe("/traj_planner/trajectory_" + std::to_string(trajectory_index), 1000, trajectory_callback);

    boost::function<void(const nav_msgs::Odometry::ConstPtr& msg)> odometry_callback =
            boost::bind(&TrajectoryController::odometry_callback, &controller, _1);
    auto odo_subscriber = n.subscribe("/lgp_car/odometry", 1000, odometry_callback);

    ros::Rate loop_rate(30);

    while (ros::ok())
    {  
      auto msgs = controller.create_control();
      const auto & ctrl = msgs.first;
      const auto & target = msgs.second;

      //static int l = 0;
      //l++;
      //if(l%5==0) ROS_INFO_STREAM("commanded velocity:" << ctrl.linear.x);

      ctrl_publisher.publish(ctrl);
      target_pose_publisher.publish(target);

      ros::spinOnce();

      loop_rate.sleep();
    }

    return 0;
}
