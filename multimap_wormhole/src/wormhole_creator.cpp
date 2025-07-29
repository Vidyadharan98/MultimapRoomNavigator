#include "rclcpp/rclcpp.hpp"
#include "my_custom_interfaces/srv/two_string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <fstream>
#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1 ;
using std::placeholders::_2 ;
 
class WormholeCreator : public rclcpp::Node 
{
public:
WormholeCreator() : Node("wormhole_creator") 
    {
        worm_hole_server_ = this->create_service<my_custom_interfaces::srv::TwoString>(
            "create_wormhole",
            std::bind(&WormholeCreator::WormHoleStamp, this , _1, _2));
            RCLCPP_INFO(this->get_logger(), " Service server started");

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
                std::bind(&WormholeCreator::callbackOdom, this, std::placeholders::_1));
    }
 
private:

    geometry_msgs::msg::PoseStamped current_pose;
    rclcpp::Service<my_custom_interfaces::srv::TwoString>::SharedPtr worm_hole_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose.pose = msg->pose.pose;
    }

    void WormHoleStamp(const my_custom_interfaces::srv::TwoString::Request::SharedPtr request,
                            const my_custom_interfaces::srv::TwoString::Response::SharedPtr response)
        {
            
            RCLCPP_INFO(this->get_logger(), " received strings %s %s ", (request->string_1).c_str() , (request->string_2).c_str());

            std::string source = (request->string_1).c_str() ;
            std::string destination = (request->string_2).c_str() ;

            std::string file_name = source + "_" + destination ;

            std::string pkg_path = ament_index_cpp::get_package_share_directory("multimap_wormhole");

            std::ofstream writeFile( pkg_path + "/maps/wormholes/" + file_name +".txt");
            if (writeFile.is_open()) {
                writeFile << current_pose.pose.position.x << std::endl;
                writeFile << current_pose.pose.position.y << std::endl;
                writeFile << current_pose.pose.position.z << std::endl;
                writeFile << current_pose.pose.orientation.x << std::endl;
                writeFile << current_pose.pose.orientation.y << std::endl;
                writeFile << current_pose.pose.orientation.z << std::endl;
                writeFile << current_pose.pose.orientation.w << std::endl;
                writeFile.close();
            } else {
                std::cerr << "Unable to open file for writing." << std::endl;
            }

        }

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WormholeCreator>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}