import argparse
from pathlib import Path
from stm_converter.parser import Parser
from stm_converter.generator import Generator

def main():
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
    structs_found, msg = parser.get_decls()

    msg_gen = Generator(structs_found, parser.struct_name, filename, parser.user_ns, msg=msg, interface_name=pkg_name)
    if msg_gen.check_existance():
        print(f"{msg_gen.msg_filename_}.msg exists\ntype: {msg_gen.interface_type_}")

    else:
        msg_gen.gen_msgs(msg_with_path)
        msg_gen.gen_type_adapter(type_adapter)
        msg_gen.gen_msg_description(msg_description)

if __name__ == "__main__":
    main()
