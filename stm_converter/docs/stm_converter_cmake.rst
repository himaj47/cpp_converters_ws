STM Converter CMake 
===================

This package provides CMake function for generating ROS 2 message files and type adapters from C++ structs.  

.. function:: convert_to_ros_msg(TARGET_NAME FILE [DEPENDENCIES <pkg1;pkg2;...>])

   Generate ROS 2 message definitions (`.msg`) and type adapters from a given C++ header file.

   **Parameters:**

   - ``TARGET_NAME`` (string)  
     Custom target name for this generation step.

   - ``FILE`` (string)  
     Path to the C++ header file containing the struct definition(s).

   - ``DEPENDENCIES`` (list, optional)  
     Additional ROS 2 interface packages this message depends on.

   **Behavior:**
   
   1. Runs the `stm_converter` generator executable to produce:
      
      - A ROS2 `msg` file under ``msg/``  
      - A type adapter header in ``<build>/type_adapters/``  
      - A YAML message description in ``<build>/msg_descriptions/``

   2. Calls ``rosidl_generate_interfaces()`` with the generated `msg` files.  
   3. Installs generated type adapters and messages into the package’s `share/` folder.

   **Example:**

   .. code-block:: cmake

      convert_to_ros_msg(
        my_struct_generator
        include/my_pkg/my_struct.hpp
        DEPENDENCIES my_custom_interfaces
      )

   After running ``colcon build``, this will produce:

   - ``msg/MyStruct.msg``  
   - ``install/my_pkg/share/my_pkg/type_adapters/MyStruct_type_adapter.hpp``  



Here is the source code for the ``convert_to_ros_msg`` function:

.. literalinclude:: ../../cpp_converter/cmake/generator.cmake
   :language: cmake
   :lines: 1-95
   :caption: generator.cmake
   :linenos:
