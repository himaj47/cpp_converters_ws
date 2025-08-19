"""ROS2 Message and type adapter generator.

This module uses Jinja2 templates to generate:
    - `msg` files from parsed message specifications.
    - Type adapter C++ header files for custom message integration.

It transforms C++ struct definitions into ROS 2 message definitions and supporting adapter code.
""" 

from jinja2 import Environment, FileSystemLoader

from ament_index_python.packages import get_package_share_directory
import ros2interface.api as interface

from rosidl_adapter.parser import MessageSpecification
import yaml
import os 

from stm_converter.utils import get_msg_fields
from stm_converter.utils import MESSAGE_FILE_EXTENSION, PRIMITIVE_TYPES

class Generator: 
    """Generates ROS 2 message files and type adapters from parsed C++ structs.

    Attributes:
        - ``env_`` (jinja2.Environment): Jinja2 environment for loading templates.
        - ``header_name_`` (str): C++ header file name.
        - ``ns`` (str): Namespace of the struct.
        - ``interface_name`` (str): Target ROS 2 interface name.
        - ``msg_content_`` (list[dict]): List of parsed message fields for each message.
        - ``msg`` (list[MessageSpecification]): Parsed ROS 2 message specifications.
        - ``struct_name`` (list[str]): C++ struct names found.
    """

    def __init__(self, struct_name, header:str, namespace, msg: MessageSpecification, interface_name):
        """Initialize the generator with message and template configuration.

        Args:
            - ``struct_name`` (list[str]): Names of C++ structs.
            - ``header`` (str): Header file name containing the definitions.
            - ``namespace`` (str): Namespace of the C++ definitions.
            - ``msg`` (list[MessageSpecification]): Parsed message specifications.
            - ``interface_name`` (str): Target ROS 2 interface name for generated files.
        """

        pathToTemplates = os.path.join(get_package_share_directory("stm_converter"), "resource/templates")
        self.env_ = Environment(loader=FileSystemLoader(pathToTemplates))

        self.header_name_ = header
        self.ns = namespace
        self.interface_name = interface_name

        # self.msg_filename_ = msg.msg_name 
        self.msg_content_ = list()
        self.interface_type_ = None
        self.msg = msg
        self.struct_name = struct_name


    def gen_msgs(self, file: str):
        """Generate `ROS msg` files from parsed message specifications.

        Uses the `message.txt` Jinja2 template to generate ROS2 msg files
        from parsed message field data.

        Args:
            - ``file`` (str): Not currently used.
        """

        template = self.env_.get_template("message.txt")

        try:
            os.mkdir("msg")
        except FileExistsError:
            print(f"file already exists!!")

        for msg in self.msg:
            self.msg_content_.append(get_msg_fields(msg))

            context = {"msg": self.msg_content_[-1]}

            # with open(filename, mode="w", encoding="utf-8") as output:
            with open("msg/" + msg.msg_name + MESSAGE_FILE_EXTENSION, mode="w", encoding="utf-8") as output:
                output.write(template.render(context))


    def gen_type_adapter(self, file: str):
        """Generate C++ type adapter headers for the generated messages.

        Uses the `type_adapter.txt` Jinja2 template to generate type adapter

        Args:
            - ``file`` (str): Output directory and prefix for the adapter file names.
        """

        msg_count = 0
        template = self.env_.get_template("type_adapter.txt")

        for msg_content in self.msg_content_:
            context = {"header": self.header_name_, 
                    "msg_file_name": self.msg[msg_count].msg_name,
                    "struct_name": self.struct_name[msg_count],
                    "msg": msg_content,
                    "namespace": self.ns, # currently assuming that all structs are under the same namespace
                    "interface_name": self.interface_name
                    }
            
            with open(file + self.struct_name[msg_count] + "_type_adapter.hpp", mode="w", encoding="utf-8") as output:
                output.write(template.render(context))
            
            msg_count += 1


    def gen_msg_description(self, file: str):
        """Generate msg descriptions (YAML) for the generated messages.

        Args:
            - ``file`` (str): Output directory and prefix for the msg description file names.
        """
        content = {}

        for msg in self.msg:
            content["header"] = self.header_name_ + '.hpp'
            content["fields"] = {}
            for field in msg.fields:
                if field.type.type not in PRIMITIVE_TYPES:
                    pass
                content["fields"].update({field.name: field.type.type})

            with open(file + self.header_name_ + '_desc.yaml', mode="w", encoding="utf-8") as output:
                    output.write(yaml.dump(content))


    def check_existance(self): # use find_content_pkg function from xml_parser
        """Check if the target message types already exist.

        This checks all interface packages to see if any of the msgs exists

        Returns:
            ``bool``: True if all messages exist, False otherwise.
        """

        is_present = False
        pkgs = list(interface.get_interface_packages().keys())

        msg_interfaces = interface.get_message_interfaces(pkgs) 

        for msg_filename_ in self.msg:  
            for pkg, msgs in msg_interfaces.items():
                if "msg/" + msg_filename_.msg_name in msgs:
                    is_present |= True
                    self.interface_type_ = f"{pkg}/msg/{msg_filename_.msg_name}"

            if not is_present:
                is_present |= False
                print(f"{msg_filename_.msg_name}.msg does not exist")

        return is_present
