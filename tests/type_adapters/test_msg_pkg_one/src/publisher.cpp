#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "msg_pkg_two/type_adapters/my_test_msg_type_adapter.hpp" 

using namespace std::chrono_literals;

using myAdaptedType = rclcpp::TypeAdapter<nspace::my_test_msg, msg_pkg_two::msg::MyTestMsg>;

class MinimalPublisher : public rclcpp::Node
{ 
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<myAdaptedType>("test_msg_pkg_one", 10); 
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      nspace::my_test_msg test_msg{};
      test_msg.a = true;
      test_msg.b = 'C';

      RCLCPP_INFO(this->get_logger(), "Publishing: 'a = %d' & 'b = %c'", test_msg.a, test_msg.b);
      publisher_->publish(test_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<myAdaptedType>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
