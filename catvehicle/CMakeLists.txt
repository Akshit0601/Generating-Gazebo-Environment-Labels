cmake_minimum_required(VERSION 3.8)
project(catvehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY meshes src urdf launch config world
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install(
#   PROGRAMS src/cmd2gazebo.py
#   DESTINATION lib/${PROJECT_NAME}/
#   )

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/status_pub.py
  src/cmd2gazebo.py
  src/spawner.py
  src/control.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
