#include "dwa.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DWA_node>();
    
    std::vector<std::pair<double, double>> waypoints= {
        {1.5, 1.5},
        {0.75, 1.5},
        {-1.5, -0.3}
    };

    for(const auto& waypoint : waypoints)
    {
        RCLCPP_INFO(node->get_logger(), "Setting goal to: (%.2f, %.2f)", waypoint.first, waypoint.second);
        node->resetPath();

        double convergence_time = node->get_convergence_time(waypoint.first, waypoint.second);
        double path_length = node->getPathLength();

        if(convergence_time >= 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Converged to goal (%.2f, %.2f) in %.2f seconds with path length %.2f meters",
                        waypoint.first, waypoint.second, convergence_time, path_length);
        //  average speed
            double average_speed = path_length / convergence_time;
            RCLCPP_INFO(node->get_logger(), "Speed %.2f | Convergence in %.2f", average_speed, convergence_time);\
        
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Failed to converge to goal (%.2f, %.2f)", waypoint.first, waypoint.second);
        }
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}