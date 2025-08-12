from jinja2 import Environment, FileSystemLoader

from ament_index_python.packages import get_package_share_directory
import ros2interface.api as interface

from rosidl_adapter.parser import MessageSpecification, PRIMITIVE_TYPES
import yaml
import os 
from stm_converter.utils import get_msg_fields, MESSAGE_FILE_EXTENSION

class Generator: 
    def __init__(self, struct_name, header:str, namespace, msg: MessageSpecification, interface_name):
        pathToTemplates = os.path.join(get_package_share_directory("stm_converter"), "resource/templates")
        self.env_ = Environment(loader=FileSystemLoader(pathToTemplates))

        self.header_name_ = header
        self.ns = namespace
        self.interface_name = interface_name

        # self.msg_filename_ = msg.msg_name 
        self.msg_content_ = tuple()
        self.interface_type_ = None
        self.msg = msg
        self.struct_name = struct_name


    def gen_msgs(self, file: str):
        # raise TypeError(f"gen_msgs() msg")
        template = self.env_.get_template("message.txt")
        
        # filename = self.msg.msg_name + ".msg"
        # if len(self.msg) == 1:
        #     filename = file

        # raise NameError(f"msgs found = {self.msg[1].msg_name}")

        for msg in self.msg:
            self.msg_content_ = get_msg_fields(msg)

            context = {"msg": self.msg_content_}

            # with open(filename, mode="w", encoding="utf-8") as output:
            with open(file + msg.msg_name + MESSAGE_FILE_EXTENSION, mode="w", encoding="utf-8") as output:
                    output.write(template.render(context))


    # def gen_type_adapter(self, file: str): # use struct name from xmlParser class
    #     template = self.env_.get_template("type_adapter.txt")
    #     context = {"header": self.header_name_, 
    #                "msg_file_name": self.msg.msg_name,
    #                "struct_name": self.struct_name,
    #                "msg": self.msg_content_,
    #                "namespace": self.ns,
    #                "interface_name": self.interface_name
    #                }
        
    #     # temp_file_name = f"{self.header_name_}_type_adapter.hpp"
    #     filename = file
        
    #     with open(filename, mode="w", encoding="utf-8") as output:
    #             output.write(template.render(context))


    # def gen_msg_description(self, file: str):
    #     filename = file
    #     content = {}

    #     content["fields"] = {}
    #     for field in self.msg.fields:
    #         if field.type.type not in PRIMITIVE_TYPES:
    #             pass
    #         content["fields"].update({field.name: field.type.type})

    #     # TODO update content path to .../share/<project_name>/msg/ if required
    #     # content["path"] = os.path.abspath(relative_path)
    #     content["path"] = os.getcwd()

    #     with open(filename, mode="w", encoding="utf-8") as output:
    #             output.write(yaml.dump(content))


    def check_existance(self): # use find_content_pkg function from xml_parser
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

