#include "dwa_node.hpp"

// #include <rclcpp/rclcpp.hpp>

// // # Messages`
// #include <geometry_msgs/msg/twist.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <visualization_msgs/msg/marker.hpp>

// // # Libraries for Mathematical Operations
// #include <cmath>
// #include <algorithm>
// #include <vector>
// #include <random>
// #include <climits>

// // # Transforms
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Transform.h>
// #include <tf2/LinearMath/Matrix3x3.h>

using namespace std;
DWA_node::DWA_node() : Node("dwa_node")
{
    // Publishers and Subscribers
    vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
    path_pub = create_publisher<visualization_msgs::msg::Marker>("/trajectory",10);
    scan_sub = create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,std::bind(&DWA_node::scan_callback, this, std::placeholders::_1));
    odom_sub = create_subscription<nav_msgs::msg::Odometry>("/odom",10, std::bind(&DWA_node::odom_callback, this, std::placeholders::_1));

    // Parameters
    max_speed = this->declare_parameter<float>("max_speed", 0.45);
    max_rotation_speed = this->declare_parameter<float>("max_rotation_speed", 4.0);
    time_step = this->declare_parameter<float>("time_step",0.1);
    declare_parameter<int>("trajectory_length", 500);
    declare_parameter<float>("tolerance", 0.25);
    declare_parameter<float>("goal_tolerance", 0.1);

    max_speed = this->get_parameter("max_speed").as_double();
    max_rotation_speed = this->get_parameter("max_rotation_speed").as_double();
    time_step = this->get_parameter("time_step").as_double();
    trajectory_length = this->get_parameter("trajectory_length").as_int();
    tolerance = this->get_parameter("tolerance").as_double();
    goal_tolerance = this->get_parameter("goal_tolerance").as_double();

    speed_dist = std::make_unique<std::uniform_real_distribution<double>>(-max_speed*1, max_speed);
    rotation_dist = std::make_unique<std::uniform_real_distribution<double>>(-max_rotation_speed, max_rotation_speed);

    seek_goal();
    timer = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DWA_node::timer_callback, this)
    );
}


void DWA_node::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom = *msg;
    // RCLCPP_INFO(this->get_logger(), "Odometry received: Position (%f, %f), Orientation (%f, %f, %f, %f)",
    //             current_odom.pose.pose.position.x,
    //             current_odom.pose.pose.position.y,
    //             current_odom.pose.pose.orientation.x,
    //             current_odom.pose.pose.orientation.y,
    //             current_odom.pose.pose.orientation.z,
    //             current_odom.pose.pose.orientation.w);
}
void DWA_node::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    current_scan = *msg;
    RCLCPP_INFO(this->get_logger(), "Laser scan received: %zu ranges", (current_scan.ranges.size()));
}

void DWA_node::seek_goal()
{
    cout << "Enter Goal Position X:";
    cin>>goal_x;
    cout << "Enter Goal Position Y:";
    cin>>goal_y;
    RCLCPP_INFO(this->get_logger(), "Goal set to: (%f,%f)", goal_x, goal_y);
    goal_converged = false;
}

vector <pair<double, double>> DWA_node::motion_ranges(double speed, double rotation_speed, double time_step, const nav_msgs::msg::Odometry& odom_data)
{
    vector <pair<double, double>> trajectory;
    double x = odom_data.pose.pose.position.x;
    double y = odom_data.pose.pose.position.y;
    
    tf2::Quaternion q(odom_data.pose.pose.orientation.x,
                    odom_data.pose.pose.orientation.y,
                    odom_data.pose.pose.orientation.z,
                    odom_data.pose.pose.orientation.w);
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);   
    int trajectory_length_temp = trajectory_length; 
        if(odd)trajectory_length_temp = trajectory_length/7;
    for(int i =0;i<trajectory_length_temp;i++)
    {
        yaw+=rotation_speed * time_step;
        x+= speed* cos(yaw) * time_step;
        y+= speed * sin(yaw) * time_step;
        // trajectory.push_back(make_pair(x,y));
        trajectory.emplace_back(make_pair(x,y));
    }
    return trajectory;
}
pair<double, double> DWA_node::path_gen()
{

    double speed = (*speed_dist)(gen);
    double rotation_speed = (*rotation_dist)(gen);
    double time_step = this->get_parameter("time_step").as_double();

    return {speed, rotation_speed};
}

void DWA_node::visualize_trajectories(const vector<vector<pair<double, double>>>& all_trajectories, int best_index)
{
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "odom";
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "trajectory";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    path_pub->publish(clear_marker);
    
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    
    for(unsigned int i = 0; i < all_trajectories.size(); i++)
    {
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "odom";
        path_marker.header.stamp = this->now();
        path_marker.ns = "trajectory";
        path_marker.id = i;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;
        path_marker.scale.x = 0.02;
        
        path_marker.lifetime = rclcpp::Duration::from_seconds(0.3); // Clear after 0.5 seconds
        
        if(i == best_index) {
            path_marker.color.r = 0.0;
            path_marker.color.g = 1.0;
            path_marker.color.b = 0.0;
            path_marker.color.a = 1.0;
            path_marker.scale.x = 0.05;
        } else {
            // Other trajectories in red
            path_marker.color.r = 1.0;
            path_marker.color.g = 0.0;
            path_marker.color.b = 0.0;
            path_marker.color.a = 0.3;
        }
        
        path_marker.points.clear();
        int point_count = 0;
        for(const auto& [x, y] : all_trajectories[i])
        {
            if(i!=best_index && point_count++ >= 100) break;
            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            path_marker.points.push_back(point);
        }
        path_pub->publish(path_marker);
    }
}

void DWA_node::clear_trajectory_markers()
{
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "odom";
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "trajectory";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    path_pub->publish(clear_marker);
}


//earler iteration of the function was only checking for the final x,y position of the trajectory, hence if the trajectory came around the back of the robot
//  we were looking at the wrong laser scan index, now we check for all points in the trajectory
double DWA_node::collision_avoidance(const vector<pair<double, double>>&trajectory, const sensor_msgs::msg::LaserScan& scan_data, const nav_msgs::msg::Odometry& current_odom, double tolerance = 0.2)
{
    if(scan_data.ranges.empty() || trajectory.empty()) {
        return 0.0;
    }

    // Get current robot orientation
    tf2::Quaternion q(current_odom.pose.pose.orientation.x,
                    current_odom.pose.pose.orientation.y,
                    current_odom.pose.pose.orientation.z,
                    current_odom.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_yaw;
    m.getRPY(roll, pitch, current_yaw);

    double movement_angle = 0.0;
    if(trajectory.size() > 1) {
        movement_angle = atan2(trajectory[1].second - trajectory[0].second,
                            trajectory[1].first - trajectory[0].first);
    }
    
    double relative_movement = movement_angle - current_yaw;
    while(relative_movement > M_PI) relative_movement -= 2*M_PI;
    while(relative_movement < -M_PI) relative_movement += 2*M_PI;

    vector<double> check_angles;
    
    if(abs(relative_movement) < M_PI/4) {
        check_angles = {0.0, M_PI/8, -M_PI/8};  // Front center and slight sides
    } 
    else if(abs(relative_movement) > 3*M_PI/4) {
        check_angles = {M_PI, M_PI - M_PI/8, M_PI + M_PI/8};  // Back center and slight sides
    }
    else if(relative_movement > 0) {
        check_angles = {M_PI/2, M_PI/3, 2*M_PI/3};  // Left side
    }
    else {
        check_angles = {-M_PI/2, -M_PI/3, -2*M_PI/3};  // Right side
    }
    
    int points_to_check = min(5, static_cast<int>(trajectory.size()));
    
    for(int i = 0; i < points_to_check; i++) {
        const auto& [x, y] = trajectory[i];
        
        for(double check_angle : check_angles) {
            int index = static_cast<int>((check_angle - scan_data.angle_min) / scan_data.angle_increment);
            
            if(index >= 0 && index < static_cast<int>(scan_data.ranges.size()) && 
            !std::isinf(scan_data.ranges[index]) && !std::isnan(scan_data.ranges[index])) {
                
                double distance_to_point = sqrt(pow(x - current_odom.pose.pose.position.x, 2) + 
                                            pow(y - current_odom.pose.pose.position.y, 2));
                
                if (scan_data.ranges[index] < distance_to_point + tolerance) {
                    return -20000;
                }
            }
        }
    }
    return 0.0;
}



pair<double, double> DWA_node::dwa_control(const nav_msgs::msg::Odometry& odom_data, const sensor_msgs::msg::LaserScan& scan_data)
{
    if(odom_data.header.stamp.sec==0 || scan_data.ranges.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Odometry or Laser scan data not available");
        return {0.0, 0.0};
    }

    double x = odom_data.pose.pose.position.x;
    double y = odom_data.pose.pose.position.y;
    tf2::Quaternion q(odom_data.pose.pose.orientation.x,
                    odom_data.pose.pose.orientation.y,
                    odom_data.pose.pose.orientation.z,
                    odom_data.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, theta;
    m.getRPY(roll, pitch, theta);
    double goal_distance = sqrt(pow(goal_x -x,2)+ pow(goal_y-y,2));

    if(goal_distance < goal_tolerance)
    {
        RCLCPP_INFO(this->get_logger(), "Goal converged");
        goal_converged = true;
        return {0.0, 0.0};
    }
    if(goal_distance < 0.5) {
        trajectory_length = min(trajectory_length, 20);
    }
    else {
        trajectory_length = this->get_parameter("trajectory_length").as_int();
    }

    vector<vector<pair<double, double>>> valid_trajectories;
    double best_score = -INT_MAX;
    double best_speed = 0.0, best_rotation_speed = 0.0;
    int best_trajectory_index = -1;
    bool stuck = true;

    for(int i=0;i<1000;i++)
    {
        auto [speed, rotation_speed] = path_gen();
        // bool odd;
        if(i%2==0)odd=false;
        else odd=true;
        vector<pair<double, double>> trajectory = motion_ranges(speed, rotation_speed, time_step, odom_data);
        
        double traj_goal_distance = sqrt(pow(goal_x - trajectory.back().first, 2) + 
                                    pow(goal_y - trajectory.back().second, 2));
        
        double traj_goal_angle = atan2(goal_y - trajectory.back().second, 
                                    goal_x - trajectory.back().first);
        double current_angle = atan2(sin(theta), cos(theta));
        double angle_diff = abs(traj_goal_angle - current_angle);
        
        double goal_distance_factor = -traj_goal_distance * 2.5;
        double goal_angle_factor = -angle_diff * 0.5;
        double collision_factor = DWA_node::collision_avoidance(trajectory, scan_data, odom_data, tolerance);
        double smoothness_factor = -abs(rotation_speed) * 0.05;
        
        if(collision_factor < 0) {
            continue;
        }
        else {
            stuck = false;
        }

        valid_trajectories.push_back(trajectory);

        double obstacle_distance_bias = 0.0;
        if(collision_factor == 0) {
            double min_obstacle_dist = 10.0;
            for(const auto& [x, y] : trajectory) {
                double angle_to_point = atan2(y - odom_data.pose.pose.position.y, 
                                            x - odom_data.pose.pose.position.x);
                double relative_angle = angle_to_point - theta;
                while(relative_angle > M_PI) relative_angle -= 2*M_PI;
                while(relative_angle < -M_PI) relative_angle += 2*M_PI;
                
                int index = static_cast<int>((relative_angle - scan_data.angle_min) / scan_data.angle_increment);
                if(index >= 0 && index < static_cast<int>(scan_data.ranges.size()) && 
                !std::isinf(scan_data.ranges[index]) && !std::isnan(scan_data.ranges[index])) {
                    min_obstacle_dist = min(min_obstacle_dist, static_cast<double>(scan_data.ranges[index]));
                }
            }
            if (min_obstacle_dist > tolerance) {
                obstacle_distance_bias = 1.0 / min_obstacle_dist;
                obstacle_distance_bias *= -0.5;
            } else {
                obstacle_distance_bias = -1000.0;
            }
        }

        double speed_bias = speed<0 ? -0.5:0;
        double score = goal_distance_factor + goal_angle_factor + collision_factor + smoothness_factor + speed_bias + obstacle_distance_bias;
        
        if(score > best_score) {
            best_score = score;
            best_speed = speed;
            best_rotation_speed = rotation_speed;
            best_trajectory_index = valid_trajectories.size() - 1;
        }
    }

    if(stuck) {
        RCLCPP_WARN(this->get_logger(), "Robot is stuck, stopping.");
        return {0.0, 0.0};
    }

    if (abs(best_rotation_speed - last_rotation_speed) > oscillation_threshold) {
        oscillation_count++;
    } else {
        oscillation_count = 0;
    }

    if (oscillation_count > 5) {
        RCLCPP_WARN(this->get_logger(), "Oscillation detected");
        return {0.0, 0.0};
    }

    last_rotation_speed = best_rotation_speed;

    visualize_trajectories(valid_trajectories, best_trajectory_index);

    return {best_speed, best_rotation_speed};
}

void DWA_node::timer_callback()
{
    if(goal_converged)
    {
        RCLCPP_INFO(this->get_logger(), "Goal already converged");
        return;
    }
    if(current_odom.header.stamp.sec==0 || current_scan.ranges.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for odometry or laser scan data");
        return;
    }

    auto [best_speed, best_rotation_speed] = dwa_control(current_odom , current_scan);

    // vector<pair<double, double>> trajectory = motion_ranges(best_speed, best_rotation_speed, time_step, current_odom);
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = best_speed;
    cmd_vel.angular.z = best_rotation_speed;

    // // publish path
    // visualization_msgs::msg::Marker path_marker;
    // path_marker.header.frame_id = "odom";
    // path_marker.header.stamp = this->now();
    // path_marker.ns = "trajectory";
    // path_marker.id = 0;
    // path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // path_marker.action = visualization_msgs::msg::Marker::ADD;
    // path_marker.pose.orientation.w = 1.0;
    // path_marker.scale.x = 0.03;
    // path_marker.color.r = 0.0;
    // path_marker.color.g = 1.0;
    // path_marker.color.b = 0.0;
    // path_marker.color.a = 1.0;
    // path_marker.points.clear();
    // for(const auto& [x, y] : trajectory)
    // {
    //     geometry_msgs::msg::Point point;
    //     point.x = x;
    //     point.y = y;
    //     point.z = 0.0; // Assuming a 2D plane
    //     path_marker.points.push_back(point);
    // }
    // path_pub->publish(path_marker);
    vel_pub->publish(cmd_vel);
    // RCLCPP_INFO(this->get_logger(), "Published velocityyy: linear.x=%f, angular.z=%f", cmd_vel.linear.x, cmd_vel.angular.z);
}
int main (int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DWA_node>());
    rclcpp::shutdown();
    return 0;
}


