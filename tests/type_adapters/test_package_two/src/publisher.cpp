#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "package_two/type_adapters/pkg_two_msg_type_adapter.hpp" 

using namespace std::chrono_literals;

using myAdaptedType = rclcpp::TypeAdapter<nspace::pkg_two_msg, package_two::msg::PkgTwoMsg>;

class MinimalPublisher : public rclcpp::Node
{ 
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<myAdaptedType>("test_package_two", 10); 
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      nspace::pkg_two_msg msg_two{};
      msg_two.c = 1.0;

      ns::pkg_one_msg msg_one;
      msg_one.a = 42;
      msg_one.b = {1, 2, 3, 4, 5};

      msg_two.d.push_back(msg_one);

      RCLCPP_INFO(this->get_logger(), "Publishing: 'c = %f' & 'a = %d'", msg_two.c, msg_one.a);
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
