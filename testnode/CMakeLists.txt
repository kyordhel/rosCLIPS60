cmake_minimum_required(VERSION 3.0.2)
project(testnode)

file(GLOB ROSCLIPS_SRC
  ${PROJECT_SOURCE_DIR}/*.cpp
)

## Declare an executable
add_executable(${PROJECT_NAME}
  ${ROSCLIPS_SRC}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${PROJECT_SOURCE_DIR}/
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  m
  pthread
)
