import argparse
from pathlib import Path
from stm_converter.xml_parser import xmlParser
from stm_converter.ros_msg_generator import ROSMsgGenerator

def main():
    desc = "stm_converter arguments"
    parser = argparse.ArgumentParser(description=desc)

    parser.add_argument("filename", help="header file")
    parser.add_argument("-ns", "--namespace", help="namespace")
    parser.add_argument("-o", "--out-description", help="message description YAML")
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
    pkg_name = args.package

    if not pkg_name:
        raise ValueError("Package name (--package) is required and was not set!")


    xml_parser = xmlParser(str(args.filename), pkg_name, namespace, dependencies)
    structs_found, msg = xml_parser.get_decls()

    msg_gen = ROSMsgGenerator(structs_found, xml_parser.struct_name, filename, xml_parser.user_ns, msg_description, msg=msg, interface_name=pkg_name)
    if msg_gen.check_existance():
        print(f"{msg_gen.msg_filename_}.msg exists\ntype: {msg_gen.interface_type_}")

    else:
        msg_gen.gen_msgs()
        msg_gen.gen_type_adapter()
        msg_gen.gen_msg_description(relative_file_path)

if __name__ == "__main__":
    main()
