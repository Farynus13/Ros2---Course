#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberCounter : public rclcpp::Node
{
    public:
        NumberCounter() : Node("number_counter")
        {
            count_ = 0;
            publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
            subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                std::bind(&NumberCounter::callbackNumber, this, std::placeholders::_1));
        }
    private:
        void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
        {
            count_ += msg->data;
            auto pub_msg = example_interfaces::msg::Int64();
            pub_msg.data = count_;
            publisher_->publish(pub_msg);
        }

        int count_;
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;          
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}