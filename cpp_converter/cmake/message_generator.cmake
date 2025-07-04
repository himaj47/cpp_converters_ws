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

    set(msg_description "${CMAKE_CURRENT_BINARY_DIR}/${basename}_desc.yaml")

    execute_process(
        COMMAND ros2 pkg prefix ${generator_pkg}
        OUTPUT_VARIABLE stm_converter_prefix
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(msg_generator_path "${stm_converter_prefix}/lib/${generator_pkg}/${generator_pkg}")

    set(cmd ${msg_generator_path} "${FILE}" --out-description "${msg_description}" --package "${PROJECT_NAME}")
    if(PKG_DEPENDENCIES)
        message(WARNING "Package Dependencies: ${PKG_DEPENDENCIES}")
        string(REPLACE ";" " " deps_str "${PKG_DEPENDENCIES}")
        list(APPEND cmd --deps "${deps_str}")
    endif()


    add_custom_command(
    OUTPUT ${msg_description}
    COMMAND ${cmd}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    
    COMMENT "Generating message description from ${FILE}" 
    )

    install(
        FILES ${msg_description}
        DESTINATION share/${PROJECT_NAME}/msg_descriptions
    )

    # TODO: update the destination
    # install(
    #     FILES ${headerFile}
    #     DESTINATION share/${PROJECT_NAME}/include
    # )
 
    add_custom_target(${TARGET_NAME}
        DEPENDS ${msg_description}
    )
endfunction()
