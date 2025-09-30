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
      - Message description (added to the registry)

   2. Calls ``rosidl_generate_interfaces()`` with the generated `msg` files.  
   3. Installs generated type adapters and messages into the package’s `include/` folder.

   **Example:**

   .. code-block:: cmake

      convert_to_ros_msg(
        msg_generator
        include/my_pkg/my_struct.hpp
        DEPENDENCIES my_custom_interfaces
      )

   After running ``colcon build``, this will produce:

   - ``msg/MyStruct.msg``  
   - ``install/my_pkg/include/type_adapters/my_struct_type_adapter.hpp``  

   **Note:**

   - Make sure to add a namespace.  
   - Make sure the struct name is the same as the header file name.  

   For example:

   .. code-block:: cpp

      #pragma once
      #include <vector>

      namespace ns {
          struct pkg_one_msg
          {
              int a;
              std::vector<int> b;
          };
      };


Here is the source code for the ``convert_to_ros_msg`` function:

.. literalinclude:: ../../cpp_converter/cmake/generator.cmake
   :language: cmake
   :lines: 1-95
   :caption: generator.cmake
   :linenos:
