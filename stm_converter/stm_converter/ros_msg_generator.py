from jinja2 import Environment, FileSystemLoader
import importlib.resources as pkg_resources

import stm_converter
import ros2interface.api as interface

from rosidl_adapter.parser import MessageSpecification, PRIMITIVE_TYPES
import yaml
import os 
from stm_converter.utils import get_msg_fields

class ROSMsgGenerator: 
    def __init__(self, structs_found, struct_name, header:str, namespace, msg_description, msg: MessageSpecification, interface_name):
        pathToTemplates = pkg_resources.files(stm_converter) / "resources/jinja_templates"
        self.env_ = Environment(loader=FileSystemLoader(pathToTemplates))

        self.structs_found_ = structs_found
        self.header_name_ = header
        self.ns = namespace
        self.interface_name = interface_name

        self.msg_filename_ = msg.msg_name 
        self.msg_content_ = tuple()
        self.interface_type_ = None
        self.msg = msg
        self.msg_description_ = msg_description
        self.struct_name = struct_name


    def gen_msgs(self):
        # raise TypeError(f"gen_msgs() msg")
        template = self.env_.get_template("message.txt")
        
        filename = self.msg.msg_name + ".msg"
        self.msg_content_ = get_msg_fields(self.msg)

        context = {"msg": self.msg_content_}

        with open(filename, mode="w", encoding="utf-8") as output:
                output.write(template.render(context))

    def gen_type_adapter(self): # use struct name from xmlParser class
        template = self.env_.get_template("type_adapter.txt")
        context = {"header": self.header_name_, 
                   "msg_file_name": self.msg.msg_name,
                   "struct_name": self.struct_name,
                   "msg": self.msg_content_,
                   "namespace": self.ns,
                   "interface_name": self.interface_name
                   }
        
        temp_file_name = f"{self.header_name_}_type_adapter.hpp"
        
        with open(temp_file_name, mode="w", encoding="utf-8") as output:
                output.write(template.render(context))


    def gen_msg_description(self, relative_path:str):
        description_file = self.msg_description_
        content = {}

        content["fields"] = {}
        for field in self.msg.fields:
            if field.type.type not in PRIMITIVE_TYPES:
                pass
            content["fields"].update({field.name: field.type.type})

        # TODO update content path to .../share/<project_name>/msg/ if required
        # content["path"] = os.path.abspath(relative_path)
        content["path"] = os.getcwd()

        with open(description_file, mode="w", encoding="utf-8") as output:
                output.write(yaml.dump(content))


    def check_existance(self): # use find_content_pkg function from xml_parser
        is_present = False
        pkgs = list(interface.get_interface_packages().keys())

        msg_interfaces = interface.get_message_interfaces(pkgs)   
        for pkg, msgs in msg_interfaces.items():
            if "msg/" + self.msg_filename_ in msgs:
                is_present = True
                self.interface_type_ = f"{pkg}/msg/{self.msg_filename_}"

        if not is_present:
            print(f"{self.msg_filename_}.msg does not exist")
            
        return is_present


                    