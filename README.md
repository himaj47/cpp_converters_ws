# C++ Struct to ROS 2 Message Converter

[![Docs](https://img.shields.io/badge/docs-online-blue)](https://himaj47.github.io/cpp_converters_doc/)

This package allow users to  **convert their C/C++ structs into ROS2 messages and generate type adapters automatically**.  
This removes the need for developers to manually define message files or boilerplate conversion logic, making it easier to use C/C++ structures directly in ROS2.

---

## 📖 Documentation

Documentation is hosted here: [cpp_converters_doc](https://himaj47.github.io/cpp_converters_doc/)

---

## 📦 Installation

Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/<your-repo>/cpp_converters.git
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
