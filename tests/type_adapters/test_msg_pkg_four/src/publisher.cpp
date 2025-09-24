#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "msg_pkg_four/type_adapters/using_num_type_adapter.hpp" 

using namespace std::chrono_literals;

using myAdaptedType = rclcpp::TypeAdapter<ns::using_num, msg_pkg_four::msg::UsingNum>;

class MinimalPublisher : public rclcpp::Node
{ 
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<myAdaptedType>("test_msg_pkg_four", 10); 
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      ns::using_num using_num{};
      using_num.number = 1.0;

      ns1::num num;
      num.num = true;
      num.arr = {1, 2, 3, 4, 5};

      using_num.arr = num;

      RCLCPP_INFO(this->get_logger(), "Publishing: 'using_num.number = %f' & 'num.num = %d'", using_num.number, num.num);
      publisher_->publish(using_num);
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
