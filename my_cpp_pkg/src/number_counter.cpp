#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class NumberCounter : public rclcpp::Node
{
    public:
        NumberCounter() : Node("number_counter")
        {
            server_ = this->create_service<example_interfaces::srv::SetBool>(
                "reset_counter", std::bind(&NumberCounter::callbackResetCounter, this, std::placeholders::_1, std::placeholders::_2));
            count_ = 0;
            publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
            subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                std::bind(&NumberCounter::callbackNumber, this, std::placeholders::_1));
        }
    private:
        void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                                  const example_interfaces::srv::SetBool::Response::SharedPtr response)
        {
            if (request->data)
            {
                count_ = 0;
                response->success = true;
                response->message = "Counter has been reset.";
            }
            else
            {
                response->success = false;
                response->message = "Counter has not been reset.";
            }
        }
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
        rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}