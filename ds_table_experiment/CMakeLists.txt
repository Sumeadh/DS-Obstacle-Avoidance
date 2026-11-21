cmake_minimum_required(VERSION 3.8)
project(ds_table_experiment)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds experiments
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/Controller.py
    scripts/target_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/plotting/visualize_results.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME visualize_results.py
)

install(
  DIRECTORY scripts/algorithms scripts/plotting
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "visualize_results.py" EXCLUDE
)

ament_package()
