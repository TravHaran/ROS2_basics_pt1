#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

/**
 * create a number_counter node
 * subscribes to the "/number" topic
 * keeps a counter variable.
 * every time a new number is received, it's added to the counter
 * the node also has a publisher on the "/number_count" topic
 * when the counter is updated, the publisher directly publishes the new value on the topic
*/

class NumberCounterNode : public rclcpp::Node 
{
public:
    NumberCounterNode() : Node("node_name"), counter_(0)
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10,
            std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
    }
 
private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg){
        counter_ += msg->data;
        auto newMsg = example_interfaces::msg::Int64();
        newMsg.data = counter_;
        publisher_->publish(newMsg);
    }
    
    int counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}