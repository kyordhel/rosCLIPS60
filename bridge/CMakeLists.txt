cmake_minimum_required(VERSION 3.0.2)
project(bridge)

file(GLOB BRIDGE_SRC
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

## Declare static library
add_library(${PROJECT_NAME} STATIC
  ${BRIDGE_SRC}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  # clips60/include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  clips60
)

# set_target_properties(bridge
#   PROPERTIES
#   OUTPUT_NAME "lib${PROJECT_NAME}.a"
#   PREFIX ""
#   SUFFIX ""
# )
