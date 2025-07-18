from rosidl_adapter.parser import MessageSpecification, Type, Field
from rosidl_adapter.parser import PRIMITIVE_TYPES, parse_message_file

import ros2interface.api as interface

import os
import yaml
import re

VECTOR_TYPE_PREFIX = "std::vector<"
VECTOR_TYPE_SUFFIX = ">"
STRUCT_NAME_SEPARATOR = "_"
DECLARATION_PREFIX = "_"
MESSAGE_FILE_EXTENSION = ".msg"
SCOPE_RESOLUTION_OPR = "::"
DEFAULT_INTERFACE_NAME = "enter_interface_name"

count = 0

def get_header_name(message_name: str):
    header = re.sub(r"(?<!^)(?=[A-Z])", "_", message_name).lower()

    return header


def generate_msg_name(msg_name: str):
    processed_msg_name = ''

    if msg_name.find(STRUCT_NAME_SEPARATOR) != -1:
        split_msg = msg_name.split(STRUCT_NAME_SEPARATOR)

        for part in split_msg:
            if part:
                processed_msg_name += part.title()
            else:
                print("error: remove first or last underscore")
                return ''
    else:
        processed_msg_name += msg_name.title()

    return processed_msg_name


def remove_namespace(typename: str):
    namespace = ''
    type_ = ''
    generated_type = ''

    parts = typename.split(SCOPE_RESOLUTION_OPR)
    print(f"typename = {typename}")

    if len(parts) == 2:
        namespace = parts[0]
        type_ = parts[1]
        generated_type = generate_msg_name(parts[1])
    
    else:
        print("illegal use of '::' operator!!")

    return namespace, generated_type, type_


def get_msg_fields(msg: MessageSpecification):
    all_fields = {}

    for field in msg.fields:
        all_fields[field.name] = [field.type.type, field.type.is_array, field.type.array_size, field.type.pkg_name]

        try:
            all_fields[field.name].append(field.msg_fields)

        except Exception as e:
            # TODO update msg_fields for complex types
            msg_fields = {}
            
            if field.type.type not in PRIMITIVE_TYPES:
                _, fields, _ = process_non_primitives(typename=field.type.type, is_array=field.type.is_array, pkg_name=field.type.pkg_name)
                msg_fields.update(fields)
                
            field.msg_fields = msg_fields
            all_fields[field.name].append(field.msg_fields)

    return all_fields


def get_fields(already_exists: bool, pkg_name: str, path:str , name: str):
    filename = ''
    if already_exists:
        filename += f"{path}/{pkg_name}/msg/{name}" + MESSAGE_FILE_EXTENSION
    else:
        filename += f"{path}/{name}" + MESSAGE_FILE_EXTENSION

    print(f"filename = {filename}")

    try:
        msg = parse_message_file(pkg_name=pkg_name, interface_filename=filename)
        fields = get_msg_fields(msg)
    except Exception as e:
        raise TypeError(f"exception: {e}\n filename = {filename}") 

    return fields


def find_context_pkg(typename: str, header_name: str=None, pkg_name: str=DEFAULT_INTERFACE_NAME, deps: list=None):
    # TODO: remove context_pkg -> no need, now checks from the dependencies
    context_pkg = ''
    pathToFile = ''
    already_exists = False
    
    # checks the dependencies
    msg = typename + MESSAGE_FILE_EXTENSION
    exists = False
    dependencies = []

    if deps: dependencies = deps
    else: dependencies.append(pkg_name)

    for pkg in dependencies:
        current_dir = os.getcwd()
        pathToPkg = current_dir + "/../../" + f"install/{pkg}/share/{pkg}/msg_descriptions"

        if typename == header_name:
            header_name = get_header_name(typename)
        
        # TODO parse the yaml file to check if msg exists 
        full_path = os.path.join(pathToPkg, header_name + "_desc" + ".yaml")
        
        if os.path.exists(full_path):
            exists |= True
            context_pkg += pkg
            # parsing the description yaml
            with open(full_path, "r") as file:
                data = file.read()
                description: dict = yaml.load(data, yaml.SafeLoader)

                # TODO update pathToFile - description[typename]["path"] or path = /install/.../share/.../msg/
                pathToFile = description["path"]          

    if not exists:
        # TODO raise error!!
        pass

    # TODO remove context_pkg and check if var "already_exists" can be replaced with var "exists"
    return context_pkg, pathToFile, already_exists


def check_if_primitve(typename: str):
    field_type = ''

    if typename in PRIMITIVE_TYPES: 
        field_type += typename

    elif typename == "double":
        field_type += "float32"

    elif typename == "u16string" or typename == "u32string":
        field_type += "wstring"

    elif typename + '32' in PRIMITIVE_TYPES:
        field_type += typename + '32'

    elif typename[-1] == 't' and typename[-2] == '_':
        type = typename.strip("_t")
        if type in PRIMITIVE_TYPES:
            field_type += type

    return field_type


def process_non_primitives(typename: str, is_array=False, pkg_name:str =DEFAULT_INTERFACE_NAME, deps: list=None):
    field_type = ''
    context_pkg = ''
    msg_fields = {}

    field_type += check_if_primitve(typename)

    if not field_type:
        field_type += typename

        # assuming namespace typed struct
        if field_type.find(SCOPE_RESOLUTION_OPR) != -1:
            namespace, field_type, typename = remove_namespace(field_type) 

            type_ = check_if_primitve(typename)
            # raise TypeError(f"field_type = {field_type} and type_ = {type_}")
            if type_:
                if is_array: type_ += "[]"
                return context_pkg, msg_fields, type_

        pkg, path, already_exists = find_context_pkg(field_type, typename, pkg_name, deps)
        context_pkg += pkg
        msg_fields = get_fields(already_exists=already_exists, pkg_name=context_pkg, path=path, name=field_type)

    if is_array: field_type += "[]"

    return context_pkg, msg_fields, field_type