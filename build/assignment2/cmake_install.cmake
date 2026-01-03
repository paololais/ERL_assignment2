# Install script for directory: /home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/install/assignment2")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2" TYPE DIRECTORY FILES
    "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/launch"
    "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/urdf"
    "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/meshes"
    "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/worlds"
    "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/rviz"
    "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/pddl"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/assignment2" TYPE EXECUTABLE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/get_plan")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/CMakeFiles/get_plan.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan_and_execute" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan_and_execute")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan_and_execute"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/assignment2" TYPE EXECUTABLE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/get_plan_and_execute")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan_and_execute" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan_and_execute")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan_and_execute"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/get_plan_and_execute")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/CMakeFiles/get_plan_and_execute.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/move_action_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/move_action_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/move_action_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/assignment2" TYPE EXECUTABLE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/move_action_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/move_action_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/move_action_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/move_action_node"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/move_action_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/CMakeFiles/move_action_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/take_picture_action_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/take_picture_action_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/take_picture_action_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/assignment2" TYPE EXECUTABLE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/take_picture_action_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/take_picture_action_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/take_picture_action_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/take_picture_action_node"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/take_picture_action_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/CMakeFiles/take_picture_action_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/detect_marker_action_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/detect_marker_action_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/detect_marker_action_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/assignment2" TYPE EXECUTABLE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/detect_marker_action_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/detect_marker_action_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/detect_marker_action_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/detect_marker_action_node"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/detect_marker_action_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/CMakeFiles/detect_marker_action_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/finalize_detection_phase_action_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/finalize_detection_phase_action_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/finalize_detection_phase_action_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/assignment2" TYPE EXECUTABLE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/finalize_detection_phase_action_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/finalize_detection_phase_action_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/finalize_detection_phase_action_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/finalize_detection_phase_action_node"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/finalize_detection_phase_action_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/CMakeFiles/finalize_detection_phase_action_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/process_last_image_action_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/process_last_image_action_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/process_last_image_action_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/assignment2" TYPE EXECUTABLE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/process_last_image_action_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/process_last_image_action_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/process_last_image_action_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/process_last_image_action_node"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/assignment2/process_last_image_action_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/CMakeFiles/process_last_image_action_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2/environment" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2/environment" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/assignment2-0.0.0-py3.12.egg-info" TYPE DIRECTORY FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_python/assignment2/assignment2.egg-info/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/assignment2" TYPE DIRECTORY FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/assignment2/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/install/assignment2/lib/python3.12/site-packages/assignment2"
      )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/assignment2")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/assignment2")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2/environment" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2/environment" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_index/share/ament_index/resource_index/packages/assignment2")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2/cmake" TYPE FILE FILES
    "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_core/assignment2Config.cmake"
    "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/ament_cmake_core/assignment2Config-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/assignment2" TYPE FILE FILES "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/utente/Documenti/Magistrale/SecondoAnno/EXP_RO/EXP_Ws/src/ERL_assignment2/build/assignment2/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
