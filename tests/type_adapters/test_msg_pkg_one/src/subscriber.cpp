#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <msg_pkg_two/type_adapters/my_test_msg_type_adapter.hpp>

using std::placeholders::_1;
using myAdaptedType = rclcpp::TypeAdapter<nspace::my_test_msg, msg_pkg_two::msg::MyTestMsg>;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<myAdaptedType>(
      "test_msg_pkg_one", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const nspace::my_test_msg & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d', '%c", msg.a, msg.b); 
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
