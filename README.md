# C/C++ Structs to ROS2 Message Converter

[![Docs](https://img.shields.io/badge/docs-online-blue)](https://himaj47.github.io/cpp_converters_doc/)

This package allow users to convert their **C/C++ structs into ROS2 messages and generate type adapters automatically**.  
This removes the need for developers to manually define message files or boilerplate conversion logic, making it easier to use C/C++ structures directly in ROS2.

---

## 📦 Installation

Clone this repository into your ROS2 workspace:
```bash
git clone https://github.com/himaj47/cpp_converters_ws.git
```

Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -y
```

Build package:
```bash
colcon build --symlink-install
source install/setup.bash
```