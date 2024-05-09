#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

/***
 * create a number_publisher node
 * publishes a number (always the same) on the "/number" topic,
 * with the existing type example_interfaces/msg/Int64
*/

class NumberPublisherNode : public rclcpp::Node 
{
public:
    NumberPublisherNode() : Node("node_name"), number_(2)
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(2), 
            std::bind(&NumberPublisherNode::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number Publisher has been started.");

    }
 
private:
    void publishNumber() {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        publisher_ ->publish(msg);
    }
    int number_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}