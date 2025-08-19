"""CLI entry point for stm_converter.

This script converts C++ struct definitions into ROS2 msg message files and generate their corresponding type adapters and optionally message description YAML files.

It uses the `Parser` class to extract message definitions from C++ headers and the
`Generator` class to produce ROS 2 artifacts based on Jinja2 templates.

Example:
    ```bash
    python3 main.py my_structs.hpp --namespace my_namespace --out-description msg_desc.yaml --out-msg msg/MyMessage.msg --out-adapter adapters/ --package my_ros_pkg --deps std_msgs sensor_msgs
    ```
"""

import argparse
from pathlib import Path
from stm_converter.parser import Parser
from stm_converter.generator import Generator

def main():
    """
    Parses command-line arguments, processes the given C++ headers,
    and generates ROS 2 message files and type adapters.

    Raises:
        ``ValueError``: If required arguments are missing.
    """

    desc = "stm_converter arguments"
    parser = argparse.ArgumentParser(description=desc)

    parser.add_argument("filename", help="header file")
    parser.add_argument("-ns", "--namespace", help="namespace")
    parser.add_argument("-o", "--out-description", help="message description YAML")
    parser.add_argument("-m", "--out-msg", help="ros message")
    parser.add_argument("-t", "--out-adapter", help="type adaption")
    parser.add_argument("-p", "--package", help="ROS package name")
    parser.add_argument("-d", "--deps", help="ROS package dependecies")

    args = parser.parse_args()

    dependencies = str(args.deps).split(" ")

    relative_file_path = str(args.filename)
    path = Path(relative_file_path)
    filename = path.stem

    namespace = ""
    if args.namespace:
        namespace = args.namespace

    msg_description = args.out_description
    msg_with_path = args.out_msg
    type_adapter = args.out_adapter
    pkg_name = args.package

    if not pkg_name:
        raise ValueError("Package name (--package) is required and was not set!")
    if not msg_description:
        raise ValueError("msg description (--out-description) is required and was not set!")
    if not msg_with_path:
        raise ValueError("msg name (--out-msg) is required and was not set!")
    if not type_adapter:
        raise ValueError("type adapter name (--out-adapter) is required and was not set!")


    parser = Parser(str(args.filename), pkg_name, namespace, dependencies)
    msg = parser.get_decls()

    generator = Generator(parser.struct_name, filename, parser.user_ns, msg=msg, interface_name=pkg_name)
    if generator.check_existance():
        print(f"{generator.msg_filename_}.msg exists\ntype: {generator.interface_type_}")

    else:
        generator.gen_msgs(msg_with_path)
        generator.gen_type_adapter(type_adapter)
        generator.gen_msg_description(msg_description)

if __name__ == "__main__":
    main()