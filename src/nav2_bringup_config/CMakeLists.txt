cmake_minimum_required(VERSION 3.8)
project(nav2_bringup_config)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  config
  behavior_trees
  waypoints
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts (with .py suffix)
install(PROGRAMS
  scripts/waypoint_navigator.py
  scripts/cmd_vel_to_uart.py
  DESTINATION lib/${PROJECT_NAME}
)

# Also install without .py suffix for ros2 run convenience
foreach(py_script cmd_vel_to_uart.py waypoint_navigator.py)
  get_filename_component(script_name ${py_script} NAME_WE)
  set(src_script ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${py_script})
  set(dst_script ${CMAKE_CURRENT_BINARY_DIR}/${script_name})
  add_custom_command(
    OUTPUT ${dst_script}
    COMMAND ${CMAKE_COMMAND} -E copy ${src_script} ${dst_script}
    DEPENDS ${src_script}
  )
  add_custom_target(create_${script_name} ALL DEPENDS ${dst_script})
  install(PROGRAMS ${dst_script} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
