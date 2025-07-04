from pygccxml import utils
from pygccxml import declarations
from pygccxml import parser

from rosidl_adapter.parser import MessageSpecification, Type, Field

from stm_converter.utils import generate_msg_name, process_non_primitives, get_msg_fields
from stm_converter.utils import DECLARATION_PREFIX, VECTOR_TYPE_PREFIX, VECTOR_TYPE_SUFFIX

class xmlParser:
    def __init__(self, filename:str, pkg_name:str, namespace="", dependencies=None):
        # Find out the c++ parser
        generator_path, generator_name = utils.find_xml_generator()

        # Configure the xml generator
        xml_generator_config = parser.xml_generator_configuration_t(
            xml_generator_path=generator_path,
            xml_generator=generator_name)
        
        self.filename = filename
        self.namespace = namespace
        self.pkg_name = pkg_name
        self.deps = dependencies
        self.ns = None
        self.user_ns = ""
        self.struct_name = ""

        self.structs = {}

        self.decls = parser.parse([filename], xml_generator_config)
        self.global_namespace = declarations.get_global_namespace(self.decls)
        self.namespaces = self.global_namespace.namespaces()

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

        for decl in self.ns.declarations:
            # TODO: check for opaque types
            # if isinstance(decl, declarations.opaque_type_t):
            #     print("opaque")
            if isinstance(decl, declarations.class_t):
                if str(decl.name).startswith(DECLARATION_PREFIX): continue
                temp = {decl.name: {}}
                self.struct_name = decl.name

                msg_name += generate_msg_name(decl.name)
                context_pkg = None

                for var in decl.variables():
                    var_type = ""

                    field_name = var.name
                    field_type = ''
                    msg_fields = None

                    # TODO: use isinstance() instead of type()
                    if type(var.decl_type) == declarations.cpptypes.int_t:
                        field_type += "int64"

                    if type(var.decl_type) == declarations.cpptypes.long_int_t:
                        field_type += "int64"
                    
                    elif type(var.decl_type) == declarations.cpptypes.float_t:
                        field_type += "float64"

                    elif type(var.decl_type) == declarations.cpptypes.bool_t:
                        field_type += "bool"

                    elif str(var.decl_type).startswith(VECTOR_TYPE_PREFIX):
                        vector_type = str(var.decl_type).strip(VECTOR_TYPE_PREFIX).strip(VECTOR_TYPE_SUFFIX)
                        context_pkg, msg_fields, field_type = process_non_primitives(vector_type, True, self.pkg_name, deps=self.deps)

                    else:
                        print(f"weird type found - {var.decl_type}")
                        context_pkg, msg_fields, field_type = process_non_primitives(typename=str(var.decl_type), pkg_name=self.pkg_name, deps=self.deps)
                    
                    field_type = Type(field_type, context_package_name=context_pkg)
                    field = Field(type_=field_type, name=field_name)
                    field.msg_fields = msg_fields
                    fields.append(field)

                    temp[decl.name].update({var.name: var_type})

                self.structs.update(temp)

        msg = MessageSpecification(pkg_name=self.pkg_name, msg_name=msg_name, fields=fields, constants=[])
        return self.structs, msg