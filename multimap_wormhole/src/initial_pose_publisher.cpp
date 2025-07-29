#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

 
class InitialPosePublisherNode : public rclcpp::Node 
{
public:
InitialPosePublisherNode() : Node("initial_pose_publisher")
    {
        initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10);
        timer_ = this->create_wall_timer(std::chrono::seconds(5),
                                        std::bind(&InitialPosePublisherNode::publishPose, this));
    }
 
private:

    void publishPose(){
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.pose.pose.position.x = -2.0;
        msg.pose.pose.position.y = 1.0;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = 0.0;
        msg.pose.pose.orientation.w = 1.0;
        initial_pose_publisher_->publish(msg);

        timer_->cancel();

    }
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisherNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}