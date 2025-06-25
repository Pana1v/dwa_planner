#ifndef DWA_NODE_HPP
#define DWA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <algorithm>
#include <vector>
#include <random>
#include <memory>
#include <utility>
#include <climits>
#include <iostream>

 #include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

class DWA_node : public rclcpp::Node
{
    public:
        DWA_node();
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        void timer_callback();
        void seek_goal();
        std::pair<double, double> dwa_control(const nav_msgs::msg::Odometry& odom_data, const sensor_msgs::msg::LaserScan& scan_data);
        std::vector<std::pair<double, double>> motion_ranges(double speed, double rotation_speed, double time_step, const nav_msgs::msg::Odometry& odom_data);
        std::pair<double, double> path_gen();
        std::chrono::steady_clock::time_point start_time;
        double collision_avoidance(const std::vector<std::pair<double, double>>& trajectory, const sensor_msgs::msg::LaserScan& scan_data, const nav_msgs::msg::Odometry& current_odom, double tolerance);
        void visualize_trajectories(const std::vector<std::vector<std::pair<double, double>>>& all_trajectories, int best_trajectory_index);
        double get_current_x() const;
        double get_current_y() const;
        double get_convergence_time(double goal_x, double goal_y);
        double getPathLength();
        bool isConverged() const { return goal_converged; }
        void resetPath();
        std::pair<double, double> get_current_position() const;
        std::vector<std::pair<double, double>> get_path_history() const;
        std::vector<std::pair<double, double>> path_history;
        bool timing_started = false;
        double total_path_length = 0.0;
        void setGoal(double x, double y);
        void clear_trajectory_markers();

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::TimerBase::SharedPtr timer;

        nav_msgs::msg::Odometry current_odom;
        sensor_msgs::msg::LaserScan current_scan;
        double goal_x, goal_y;
        bool goal_converged = false;

        double max_speed, max_rotation_speed, tolerance, goal_tolerance, time_step;
        int trajectory_length;

        double last_rotation_speed = 0.0;
        int oscillation_count = 0;
        double oscillation_threshold = 0.3;
        bool odd = false;

        // Random number geneators

        std::random_device rd;
        std::mt19937 gen;
        std::unique_ptr<std::uniform_real_distribution<double>> speed_dist;
        std::unique_ptr<std::uniform_real_distribution<double>> rotation_dist;
    private:
        double collision_avoidance_factor = -1000.0;
        double goal_distance_factor = 0.0;
        double goal_angle_factor = 0.0;
        double smoothness_factor = 0.0;
        double obstacle_distance_bias = 0.0;

        int trajectory_length_temp = 500; // Default value, can be adjusted based on goal distance
};

#endif
