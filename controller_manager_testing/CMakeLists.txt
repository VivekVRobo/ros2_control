cmake_minimum_required(VERSION 3.16)
project(controller_manager_testing)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(controller_manager_testing)

ament_package()
