cmake_minimum_required(VERSION 3.8)
project(robot)

# Znajdź ament_cmake, potrzebne do budowania pakietów ROS 2
find_package(ament_cmake REQUIRED)

# Skopiuj foldery launch i config do katalogu share pakietu
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Zakończenie konfiguracji pakietu
ament_package()
