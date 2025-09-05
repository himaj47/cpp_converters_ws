#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "package_two/type_adapters/pkg_two_msg_type_adapter.hpp" 

using std::placeholders::_1;
using myAdaptedType = rclcpp::TypeAdapter<nspace::pkg_two_msg, package_two::msg::PkgTwoMsg>;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<myAdaptedType>(
      "test_package_two", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const nspace::pkg_two_msg & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "pkg_two_msg:");
      RCLCPP_INFO(this->get_logger(), "  c = %f", msg.c);

      for (size_t i = 0; i < msg.d.size(); i++) {
        const auto & inner = msg.d[i];
        std::ostringstream oss;
        for (auto val : inner.b) {
          oss << val << " ";
        }

        RCLCPP_INFO(this->get_logger(),
          "  d[%zu]: a = %d, b = [%s]",
          i, inner.a, oss.str().c_str());
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
