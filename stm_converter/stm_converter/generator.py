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


def check_for_extras(msg_content: dict, template_env: Environment, file_path: str):
    extras = ""
    template = template_env.get_template("extras.txt")
    extras_properties = {}

    for field, properties in msg_content.items():
        if properties[0] not in PRIMITIVE_TYPES: # update the index 

            # find msg description in registry
            non_primitive = properties[0]

            with open(file_path, 'r') as f:
                data = yaml.safe_load(f) or {}

                if not data:
                    raise ValueError(f"registry is empty!!")
                
                try: 
                    header = data[non_primitive]["header"].split(".")[0] + "_type_adapter.hpp"
                    package = data[non_primitive]["package"]

                    # mapping of msg name to its namespace and struct name
                    temp = {non_primitive: [data[non_primitive]["namespace"], data[non_primitive]["struct_name"]]}
                    extras_properties.update(temp)

                    context = {"header": header,
                               "pkg": package}
                    
                    extras += template.render(context) + "\n"

                except KeyError:
                    print(f"msg description for {non_primitive} does not exists in the registry!!")

    return extras, extras_properties


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

        # package name
        self.interface_name = interface_name

        # self.msg_filename_ = msg.msg_name 
        self.msg_content_ = list()
        self.interface_type_ = None
        self.msg = msg
        self.struct_name = struct_name

        self.non_primitive_info_ = {}


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
        # path to registry
        file_path = os.path.expanduser("~/.registry.yaml")

        # include headers
        # includes_template = self.env_.get_template("type_adapter/includes/includes.txt")
        includes_template = self.env_.get_template("includes.txt")
        include_content = []
        struct_count = 0

        for msg_content in self.msg_content_: 

            # check for any extra includes
            extras, extras_properties = check_for_extras(msg_content, self.env_, file_path)
            self.non_primitive_info_.update(extras_properties)

            context = {"interface_name": self.interface_name,
                       "struct_name": self.struct_name[struct_count],
                       "header": self.header_name_,
                       "extras": extras}

            struct_count += 1

            temp = includes_template.render(context)
            include_content.append(temp)


        non_array_types = self.env_.get_template("non_array_types.txt")
        adapter = self.env_.get_template("adapter.txt")

        loop = self.env_.get_template("loop.txt")
        push_back = self.env_.get_template("push_back.txt")
        np_push_back = self.env_.get_template("np_push_back.txt")

        to_ros_msg = self.env_.get_template("adapter__to_ros.txt")
        to_custom = self.env_.get_template("adapter__to_custom.txt")

        type_adapter = self.env_.get_template("type_adapter1.txt")

        # convertion block
        type_adapters = []
        msg_count = 0

        for msg_content in self.msg_content_:
            content = ""
            non_primitives__to_ros = ""
            non_primitives__to_custom = ""

            for field, info in msg_content.items():
                temp = ""

                # non array primitive types
                if not info[1] and not info[-1]:
                    context = {"field_name": field}
                    temp = non_array_types.render(context) + "\n"

                # non array non primitive types
                elif not info[1]:
                    context = {"namespace": self.non_primitive_info_[info[0]][0],
                               "struct": self.non_primitive_info_[info[0]][1],
                               "pkg": info[3],
                               "msg": info[0],
                               "field_name": field}

                    temp = adapter.render(context) + "\n"

                # array primitive types
                elif info[1] and not info[-1]:
                    context = {"field_name": field}
                    temp = push_back.render(context)

                    context = {"field_name": field,
                               "content": temp}
                    temp = loop.render(context)

                # array non primitve types
                else:
                    context = {"pkg": info[3],
                               "msg": info[0],
                               "namespace": self.non_primitive_info_[info[0]][0],
                               "struct": self.non_primitive_info_[info[0]][1],
                               }
                    
                    temp_render = to_ros_msg.render(context)

                    push_back_context = {"field_name": field}
                    temp_push_back = np_push_back.render(push_back_context)
                    temp_render += "\n" + temp_push_back

                    loop_context = {"field_name": field,
                               "content": temp_render}
                    
                    temp__to_ros = loop.render(loop_context)


                    temp_render = to_custom.render(context)
                    temp_render += "\n" + temp_push_back

                    loop_context["content"] = temp_render
                    temp__to_custom = loop.render(loop_context)

                    non_primitives__to_ros += temp__to_ros
                    non_primitives__to_custom += temp__to_custom

                content += temp
            
            context = {"msg": self.msg[msg_count].msg_name,
                       "struct_name": self.struct_name[msg_count],
                       "namespace": self.ns, # currently assuming that all structs are under the same namespace
                       "interface_name": self.interface_name,
                       "primitives": content,
                       "to_ros_msg": non_primitives__to_ros,
                       "to_custom": non_primitives__to_custom,
                       "includes": include_content[msg_count]}
            
            full_content = type_adapter.render(context)
            type_adapters.append(full_content)
            msg_count += 1


        msg_count = 0
        template = self.env_.get_template("type_adapter.txt")

        for content in type_adapters:
            
            with open(file + self.struct_name[msg_count] + "_type_adapter.hpp", mode="w", encoding="utf-8") as output:
                output.write(content)
            
            msg_count += 1


    def gen_msg_description(self, file: str):
        """Generate msg descriptions (YAML) for the generated messages.

        Args:
            - ``file`` (str): Output directory and prefix for the msg description file names.
        """

        # create registry if doesn't exists
        file_path = os.path.expanduser("~/.registry.yaml")

        if not os.path.exists(file_path):
            with open(file_path, "w") as f:
                yaml.dump({}, f)

        else:
            print(f"Registry already exists: {file_path}")

        # read registry
        with open(file_path, "r") as f:
            data = yaml.safe_load(f) or {}

        count = 0
        for msg in self.msg:
            content = {msg.msg_name: {}}
            content[msg.msg_name].update({"package": self.interface_name})
            content[msg.msg_name].update({"header": self.header_name_ + '.hpp'})
            content[msg.msg_name].update({"fields": {}})

            for field in msg.fields:
                if field.type.type not in PRIMITIVE_TYPES:
                    pass
                content[msg.msg_name]["fields"].update({field.name: field.type.type})
            
            content[msg.msg_name].update({"namespace": self.ns})
            content[msg.msg_name].update({"struct_name": self.struct_name[count]})

            data.update(content)
            count += 1

        with open(file_path, 'w') as f:
            f.write(yaml.dump(data))


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
