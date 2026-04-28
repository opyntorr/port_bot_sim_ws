cmake_minimum_required(VERSION 3.8)
project(mi_proyecto_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

add_executable(planificador_rrt src/planificador_rrt.cpp)
target_include_directories(planificador_rrt PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(planificador_rrt ${OpenCV_LIBS})
ament_target_dependencies(planificador_rrt rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  planificador_rrt
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  mi_proyecto_sim/detector_aruco.py
  mi_proyecto_sim/generador_mapa.py
  mi_proyecto_sim/planificador_astar.py
  mi_proyecto_sim/control_trayectoria.py
  mi_proyecto_sim/filtro_lidar.py
  mi_proyecto_sim/slam_occupancy_grid.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  models
  worlds
  launch
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
