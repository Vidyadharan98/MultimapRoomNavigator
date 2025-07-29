#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>
#include "my_custom_interfaces/action/room_navigation.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <memory>
#include <chrono>
#include <fstream>

using RoomNavigation = my_custom_interfaces::action::RoomNavigation;
using RoomNavigationGoalHandle = rclcpp_action::ServerGoalHandle<RoomNavigation>;

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

using namespace std::placeholders ;
 
class MultiRoomNavigatorNode : public rclcpp::Node 
{
public:
    MultiRoomNavigatorNode() : Node("multi_room_navigator_node") 
    {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cmd_vel_server_ = rclcpp_action::create_server<RoomNavigation>(
            this,
            "navigate_to_room" , 
            std::bind(&MultiRoomNavigatorNode::goal_callback, this, _1 , _2),
            std::bind(&MultiRoomNavigatorNode::cancel_callback, this, _1),
            std::bind(&MultiRoomNavigatorNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
        );
        RCLCPP_INFO(this->get_logger(), "Action server has been started");

        map_load_client = create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");     

        nav_node_ = std::make_shared<rclcpp::Node>("nav_helper_node");
        
    }
 
private:

    std::string map_name;
    std::string current_map {"hall"};
    float goal_x{0} , goal_y{0} ,  goal_yaw_z{0} ,goal_yaw_w{0} ;
    rclcpp_action::Server<RoomNavigation>::SharedPtr cmd_vel_server_ ;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_load_client;
    rclcpp::Node::SharedPtr nav_node_;
    
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RoomNavigation::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received a goal");

        RCLCPP_INFO(this->get_logger(), "Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE ;
    }

    rclcpp_action::CancelResponse cancel_callback(
       const std::shared_ptr<RoomNavigationGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), " Received cancel request ");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT ;
    }

    void handle_accepted_callback(
        const std::shared_ptr<RoomNavigationGoalHandle> goal_handle)
    {
        map_name = goal_handle->get_goal()->map_name;

        std::cout << " current map : " << current_map << std::endl;

        if (map_name == current_map){
            RCLCPP_INFO(this->get_logger(), "Executing the goal");
            execute_goal(goal_handle);
        }
        else if ((map_name == "hall") || (current_map == "hall")){

            teleport(current_map, map_name);
            execute_goal(goal_handle);

        }
        else if ((map_name != "hall") && (current_map != "hall")){
            teleport(current_map, "hall");
            teleport("hall", map_name);
            execute_goal(goal_handle);
        }
        
        
    }

    bool send_nav_goal(double x, double y, double yaw_z=0, double yaw_w=1) {
       
        auto nav_client = rclcpp_action::create_client<NavigateToPose>(nav_node_, "navigate_to_pose");
        if (!nav_client->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 server not available");
            return false;
        }
    
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.orientation.z = yaw_z;
        goal.pose.pose.orientation.w = yaw_w;
    
        auto send_goal_future = nav_client->async_send_goal(goal);
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(nav_node_);
    
        if (exec.spin_until_future_complete(send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
            exec.remove_node(nav_node_);
            return false;
        }
    
        auto goal_handle = send_goal_future.get();
        if (!goal_handle) {
            exec.remove_node(nav_node_);
            return false;
        }
    
        auto result_future = nav_client->async_get_result(goal_handle);
        if (exec.spin_until_future_complete(result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            exec.remove_node(nav_node_);
            return false;
        }
    
        exec.remove_node(nav_node_);
    
        auto result = result_future.get();
        return result.code == rclcpp_action::ResultCode::SUCCEEDED;

    }


    void execute_goal(const std::shared_ptr<RoomNavigationGoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        bool nav_success = send_nav_goal(goal->goal_x,goal->goal_y);

        auto result = std::make_shared<RoomNavigation::Result>();

        if (nav_success) {
            RCLCPP_INFO(this->get_logger(), "Navigation completed");
            result->action_result = "Navigation completed";
            goal_handle->succeed(result);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Navigation failed");
            result->action_result = "Navigation failed";
            goal_handle->abort(result);
        }
    }

    void change_map(std::string source , std::string destination){

        while (!map_load_client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger(), " Waiting for the server to be up .... ");
        }

        RCLCPP_WARN(this->get_logger(), " Changing map from %s to %s ", source.c_str() , destination.c_str());

        auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();

        std::cout << " map_name  " << destination << std::endl;

        std::string pkg_path = ament_index_cpp::get_package_share_directory("multimap_wormhole");

        request->map_url = pkg_path + "/maps/"+ destination +".yaml";

        current_map = map_name;

        auto future = map_load_client->async_send_request(request);       

    }

    void teleport(std::string source , std::string destination){

        std::string pkg_path = ament_index_cpp::get_package_share_directory("multimap_wormhole");

        std::ifstream file(pkg_path + "/maps/wormholes/"+  source + "_" + destination +".txt");

        if (!file.is_open()) {
            std::cerr << "Failed to open file.\n";
            return ;
        }

        std::string line;
        int current_line = 0;

        while (std::getline(file, line)) {
            current_line++;
            if (current_line == 1) {
                goal_x = std::stof(line);
                std::cout << "goal_x: " << goal_x << std::endl;
            }
            else if(current_line == 2){
                goal_y = std::stof(line);
                std::cout << "goal_y: " << goal_y << std::endl;
            }
            else if(current_line == 3){
                goal_yaw_z = std::stof(line);
                std::cout << "goal_yaw_z: " << goal_yaw_z << std::endl;
            }
            else if(current_line == 4){
                goal_yaw_w = std::stof(line);
                std::cout << "goal_yaw_w: " << goal_yaw_w << std::endl;
            }
        
        }
        file.close();
        bool nav_result = send_nav_goal(goal_x,goal_y);
        change_map(source , destination);
    }

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiRoomNavigatorNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}