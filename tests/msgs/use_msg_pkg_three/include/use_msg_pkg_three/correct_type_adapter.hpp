#pragma once

#include <rclcpp/type_adapter.hpp>
#include "use_msg_pkg_three/msg/use_num.hpp"
#include "use_msg_pkg_three/headers/use_num.hpp"

#include "msg_pkg_three/msg/num.hpp"
#include "msg_pkg_three/headers/num.hpp"

template<>
struct rclcpp::TypeAdapter<nspace::use_num, use_msg_pkg_three::msg::UseNum>
{
  using is_specialized = std::true_type;
  using custom_type = nspace::use_num_again;
  using ros_message_type = use_msg_pkg_three::msg::UseNum;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.number = source.number;
    destination.data.clear();
    destination.data.reserve(source.data.size());

    for (const auto & elem : source.data) {
      msg_pkg_three::msg::Num msg_elem;
      msg_elem.num = elem.num;
      msg_elem.arr = elem.arr;
      destination.data.push_back(msg_elem);
    }
  }

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination.number = source.number;
    destination.data.clear();
    destination.data.reserve(source.data.size());

    for (const auto & msg_elem : source.data) {
      ns1::num custom_elem;
      custom_elem.num = msg_elem.num;
      custom_elem.arr = msg_elem.arr;
      destination.data.push_back(custom_elem);
    }
  }
};
