include("${CMAKE_CURRENT_LIST_DIR}/generate_msg_name.cmake")
find_package(rosidl_default_generators REQUIRED)

function(convert_to_ros_msg TARGET_NAME FILE)
    set(generator_pkg stm_converter)

    set(options)
    set(one_value_keywords)
    set(deps DEPENDENCIES)

    cmake_parse_arguments(PARSE_ARGV 0 PKG 
        "${options}" "${one_value_keywords}" "${deps}"
    )
    
    get_filename_component(basename ${FILE} NAME_WE)
    get_filename_component(headerFile ${FILE} NAME)

    set(description_file "${basename}_desc.yaml")
    set(type_adapter_file "${basename}_type_adapter.hpp")

    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/msg")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/msg_descriptions")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/type_adapters")

    set(msg_description "${CMAKE_CURRENT_BINARY_DIR}/msg_descriptions/${basename}_desc.yaml")
    set(type_adapter "${CMAKE_CURRENT_BINARY_DIR}/type_adapters/")

    snake_to_pascal("${basename}" out_string)
    
    # set(ros_msg "${CMAKE_CURRENT_SOURCE_DIR}/msg/${out_string}.msg")
    set(ros_msg "${CMAKE_CURRENT_SOURCE_DIR}/msg/")
    set(formatted_msg "${CMAKE_CURRENT_BINARY_DIR}:msg/${out_string}.msg")
    message(WARNING "formatted_msg: ${formatted_msg}")

    execute_process(
        COMMAND ros2 pkg prefix ${generator_pkg}
        OUTPUT_VARIABLE stm_converter_prefix
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(msg_generator_path "${stm_converter_prefix}/lib/${generator_pkg}/${generator_pkg}")

    set(cmd ${msg_generator_path} "${FILE}" --out-description "${msg_description}" --out-msg "${ros_msg}" --out-adapter "${type_adapter}" --package "${PROJECT_NAME}")

    if(PKG_DEPENDENCIES)
        message(WARNING "Package Dependencies: ${PKG_DEPENDENCIES}")
        string(REPLACE ";" " " deps_str "${PKG_DEPENDENCIES}")
        list(APPEND cmd --deps "${deps_str}")
    endif()

    set(msg_src "${CMAKE_CURRENT_SOURCE_DIR}/msg/${out_string}.msg")

    execute_process(
        COMMAND ${cmd}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )

    set(MSG_FILES "")
    set(TYPE_ADAPTERS "")

    file(GLOB_RECURSE MSG_FILES_RECURSIVE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
    file(GLOB_RECURSE TYPE_ADAPTERS_RECURSIVE "${CMAKE_CURRENT_BINARY_DIR}/type_adapters/*_type_adapter.hpp")

    list(APPEND MSG_FILES ${MSG_FILES_RECURSIVE})
    list(APPEND TYPE_ADAPTERS ${TYPE_ADAPTERS_RECURSIVE})

    string(REPLACE ";" " " deps_str "${MSG_FILES}")
    string(REPLACE ";" " " deps_str "${TYPE_ADAPTERS}")

    message(WARNING "msg files: ${MSG_FILES}")
    message(WARNING "type adapter files: ${TYPE_ADAPTERS}")


    add_custom_target(${TARGET_NAME})

    rosidl_generate_interfaces(${PROJECT_NAME}
        ${MSG_FILES}
        DEPENDENCIES ${PKG_DEPENDENCIES}
    )

    # install(
    #     FILES ${msg_description}
    #     DESTINATION share/${PROJECT_NAME}/msg_descriptions
    # )

    install(
        FILES ${TYPE_ADAPTERS}
        DESTINATION share/${PROJECT_NAME}/type_adapters
    )

    install(
        FILES ${MSG_FILES}
        DESTINATION share/${PROJECT_NAME}/msg
    )

endfunction()