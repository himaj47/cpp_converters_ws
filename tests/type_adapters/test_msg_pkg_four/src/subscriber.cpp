#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "msg_pkg_four/type_adapters/using_num_type_adapter.hpp" 

using std::placeholders::_1;
using myAdaptedType = rclcpp::TypeAdapter<ns::using_num, msg_pkg_four::msg::UsingNum>;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<myAdaptedType>(
      "test_msg_pkg_four", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const ns::using_num & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "msg_pkg_four:");
      RCLCPP_INFO(this->get_logger(), "  number = %f", msg.number);
      RCLCPP_INFO(this->get_logger(), "  msg.arr.num = %d", msg.arr.num);

      for (size_t i = 0; i < msg.arr.arr.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "  msg.arr.num = %d", msg.arr.arr[i]);
      }
    }
    rclcpp::Subscription<myAdaptedType>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
