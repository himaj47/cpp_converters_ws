from pygccxml import utils
from pygccxml import declarations
from pygccxml import parser

from rosidl_adapter.parser import MessageSpecification, Type, Field
import ros2interface.api as interface

from stm_converter.utils import generate_msg_name, process_non_primitives, get_msg_fields
from stm_converter.utils import DECLARATION_PREFIX, VECTOR_TYPE_PREFIX, VECTOR_TYPE_SUFFIX

class Parser:
    def __init__(self, filename:str, pkg_name:str, namespace="", dependencies=None):
        # Find out the c++ parser
        generator_path, generator_name = utils.find_xml_generator()

        # Configure the xml generator
        xml_generator_config = parser.xml_generator_configuration_t(
            xml_generator_path=generator_path,
            xml_generator=generator_name,
            cflags="-Wno-pragma-once-outside-header")
        
        self.filename = filename
        self.namespace = namespace
        self.pkg_name = pkg_name

        # don't need this anymore
        self.deps = dependencies

        self.ns = None
        self.user_ns = ""
        self.struct_name = list()

        self.msg_interfaces = None

        self.decls = parser.parse([filename], xml_generator_config)
        self.global_namespace = declarations.get_global_namespace(self.decls)
        self.namespaces = self.global_namespace.namespaces()

        if dependencies != ["None"]:
            try:
                self.msg_interfaces = interface.get_message_interfaces(dependencies)
            except Exception as e:
                raise NameError(f"check dependencies!! {e}")
        
        self.builtin_interface_pkgs = interface.get_interface_packages()
        
        # if namespace provided by the user
        if namespace:
            self.ns = self.global_namespace.namespace(namespace)

        else:
            for ns in self.namespaces:
                self.user_ns = ns.name

            self.ns = self.global_namespace.namespace(self.user_ns)

    def get_decls(self):
        msg_name = ''
        fields = []
        msgs = []
        test_canonical_types = []
        test_msg_names = []

        for decl in self.ns.declarations:
            # TODO: check for opaque types
            # if isinstance(decl, declarations.opaque_type_t):
            #     print("opaque")
            if isinstance(decl, declarations.class_t):
                if str(decl.name).startswith(DECLARATION_PREFIX): continue
                temp = {decl.name: {}}
                self.struct_name.append(decl.name)

                msg_name = generate_msg_name(decl.name)
                test_msg_names.append(msg_name)

                test_msg_fields = []
                context_pkg = None

                for var in decl.variables():
                    var_type = ""

                    field_name = var.name
                    field_type = ''
                    msg_fields = None

                    # var.decl_type might be any typedef or using type
                    # We need the underlying canoncical type
                    canonical_type = declarations.type_traits.remove_alias(var.decl_type)

                    # TODO: use isinstance() instead of type()
                    if type(canonical_type) == declarations.cpptypes.char_t:
                        field_type = "char"

                    elif type(canonical_type) == declarations.cpptypes.bool_t:
                        field_type = "bool"

                    elif type(canonical_type) == declarations.cpptypes.signed_char_t:
                        field_type = "int8"

                    elif type(canonical_type) == declarations.cpptypes.unsigned_char_t:
                        field_type = "uint8"

                    elif type(canonical_type) == declarations.cpptypes.short_int_t:
                        field_type = "int16"

                    elif type(canonical_type) == declarations.cpptypes.short_unsigned_int_t:
                        field_type = "uint16"    

                    elif type(canonical_type) == declarations.cpptypes.int_t:
                        field_type = "int32"

                    elif type(canonical_type) == declarations.cpptypes.unsigned_int_t:
                        field_type = "uint32"

                    elif type(canonical_type) == declarations.cpptypes.long_int_t:
                        field_type = "int64"

                    elif type(canonical_type) == declarations.cpptypes.long_unsigned_int_t:
                        field_type = "uint64"
                    
                    elif type(canonical_type) == declarations.cpptypes.float_t:
                        field_type = "float32"

                    elif type(canonical_type) == declarations.cpptypes.double_t:
                        field_type = "float64"

                    elif str(canonical_type).startswith(VECTOR_TYPE_PREFIX):
                        vector_type = str(canonical_type).strip(VECTOR_TYPE_PREFIX).strip(VECTOR_TYPE_SUFFIX)
                        context_pkg, msg_fields, field_type = process_non_primitives(vector_type, True, self.pkg_name, deps=self.deps)

                    else:
                        print(f"weird type found - {var.decl_type} - canonical type - {canonical_type}")
                        context_pkg, msg_fields, field_type = process_non_primitives(typename=str(var.decl_type), pkg_name=self.pkg_name, deps=self.deps, msg_interfaces=self.msg_interfaces, builtin_interfaces=self.builtin_interface_pkgs)

                    test_canonical_types.append(field_type)
                    field_type = Type(field_type, context_package_name=context_pkg)
                    field = Field(type_=field_type, name=field_name)
                    field.msg_fields = msg_fields
                    fields.append(field)

                    test_msg_fields.append(msg_fields)

                    temp[decl.name].update({var.name: var_type})

            msg = MessageSpecification(pkg_name=self.pkg_name, msg_name=msg_name, fields=fields, constants=[])
            msgs.append(msg)
            fields = []
        return msgs