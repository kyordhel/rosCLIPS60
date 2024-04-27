cmake_minimum_required(VERSION 3.0.2)
project(rosclips)

file(GLOB ROSCLIPS_SRC
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

## Declare an executable
add_executable(${PROJECT_NAME}
  ${ROSCLIPS_SRC}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  clips60/include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  clips60
  ${catkin_LIBRARIES}
  m
  pthread
)

# set_target_properties(clips60
#   PROPERTIES
#   OUTPUT_NAME "lib${PROJECT_NAME}.so"
#   PREFIX ""
#   SUFFIX ""
# )
