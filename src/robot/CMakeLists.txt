cmake_minimum_required(VERSION 3.8)
project(robot)

# Find ament_cmake, which provides the necessary build tools for ROS 2.
find_package(ament_cmake REQUIRED)

# The install() command is crucial.
# It tells the build system to copy the entire 'launch' directory
# to the 'share' folder of your package's installation directory.
install(DIRECTORY launch
  DESTINATION share/${my_robot}
)

# This command marks the end of the ament_cmake build rules.
ament_package()