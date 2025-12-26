cmake_minimum_required(VERSION 3.8)
project(husky_panda_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# This block installs the launch, config, and world files so ROS can find them
install(DIRECTORY
  config
  launch
  models
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/mobile_pick_place.py
  src/husky_blue_align.py
  src/visual_pick.py
  src/depth_grasp.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()