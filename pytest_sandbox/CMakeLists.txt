cmake_minimum_required(VERSION 3.8)
project(pytest_sandbox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  
  # Add launch tests
  add_launch_test(test/test_chatter_simple.py)
  add_launch_test(test/test_chatter_launch.py)
  add_launch_test(test/test_turtlesim.py)
  
  # Add regular tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(unit_tests test/test_unit.py)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()