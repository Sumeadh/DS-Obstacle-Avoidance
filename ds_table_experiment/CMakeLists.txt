cmake_minimum_required(VERSION 3.8)
project(ds_table_experiment)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/Controller.py
    scripts/AddTable.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/algorithms
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
