#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// #include "msg_pkg_one/type_adapters/type_from_one_type_adapter.hpp"  

// #include "msg_test/msg/test_msg.hpp"
// #include "msg_pkg_two/msg/my_test_msg.hpp" 
// #include "msg_pkg_one/msg/type_from_one.hpp"
#include "msg_pkg_two/type_adapters/my_test_msg_type_adapter.hpp" 
// #include "type_from_one.hpp"
// #include "msg_pkg_two/headers/my_test_msg.hpp"

// using myAdaptedType = rclcpp::TypeAdapter<nspace::my_test_msg, msg_pkg_two::msg::MyTestMsg>;

// class TypeAdapterPublisher : public rclcpp::Node
// {
// public:
//     TypeAdapterPublisher()
//     : Node("type_adapter_publisher")
//     {
//         pub_ = this->create_publisher<myAdaptedType>("type_from_one_topic", 10);

//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(1),
//             std::bind(&TypeAdapterPublisher::publish_msg, this));
//     }

// private:
//     void publish_msg()
//     {
//         nspace::my_test_msg msg{};
//         msg.a = true;
//         msg.b = 'X';
//         // msg.c = 3.14f;
//         // msg.d = 6.28;
//         // msg.e = -42;
//         // msg.f = 255;
//         // msg.g = -123;
//         // msg.h = 456;
//         // msg.i = -789;
//         // msg.j = 123456;
//         // msg.k = -987654321;
//         // msg.l = 987654321;
//         // msg.m = "Hello ROS 2 with type adapter!";
//         // msg.n = u"UnicodeTest";

//         RCLCPP_INFO(this->get_logger(), "Publishing type_from_one: a=%d, b=%c", msg.a, msg.b);
//         pub_->publish(msg);
//     }

//     rclcpp::Publisher<myAdaptedType>::SharedPtr pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char* argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<TypeAdapterPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }

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
