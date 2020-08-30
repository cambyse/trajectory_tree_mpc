#include <control_tree/ros/obstacle_common.h>

#include <fstream>

#include <ros/package.h>
#include <ros/console.h>

#include <chrono>

void log_to_file(std::ofstream & ofs, ros::NodeHandle & n, double car_x, int i, double cost, double time)
{
    int n_obstacles, n_non_obstacles;
    n.getParam("/n_obstacles", n_obstacles);
    n.getParam("/n_non_obstacles", n_non_obstacles);

    double p_obstacle;
    n.getParam("p_obstacle", p_obstacle);

    std::stringstream ss;

    ss << "N:" << n_non_obstacles + n_non_obstacles << " Avg dist between obstacles:" << car_x / (n_non_obstacles + n_non_obstacles) << " Probability:" << p_obstacle << " Avg cost:" << cost << " time(ms):" << time * 1000 << " " << i << " iterations";

    ROS_INFO_STREAM(ss.str());
    ofs << ss.str() << std::endl;
}

std::string filename(const std::string & name, double p_obstacle)
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/camille/Phd/Paper/RSS/plots/gen_obstacles/data-" << std::to_string(p_obstacle) << "-" << name << "-" << t << ".txt";
    return ss.str();
}

visualization_msgs::Marker create_obstacle_marker(double x, double y, double sx, double sy, double sz, double alpha, int id)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.id = std::hash<std::string>()("obstacle_" + std::to_string(id));
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.5 * sz;
    marker.scale.x = sx; // diameter
    marker.scale.y = sy;
    marker.scale.z = sz;
    marker.color.a = alpha;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
}

visualization_msgs::Marker create_collision_marker(double x, double y, double sx, double sy, double sz, double alpha, int id)
{
    visualization_msgs::Marker marker;

    const double m = 0.5; // margin
    double d = sx * sx / (8 * m) - sy / 2 + m / 2;
    double diameter = 2 * (d + sy / 2 + m);

    marker.header.frame_id = "map";
    marker.id = std::hash<std::string>()("collision_" + std::to_string(id));
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y + (y > 0 ? d : -d);
    marker.pose.position.z = 0.5 * 0.01;
    marker.scale.x = diameter; // diameter
    marker.scale.y = diameter;
    marker.scale.z = 0.01;
    marker.color.a = alpha > 0.01 ? 1.0 : 0.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
}
