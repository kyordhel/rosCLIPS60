cmake_minimum_required(VERSION 3.0.2)
project(clipscontrol)

find_package(Curses REQUIRED)
# add_definitions(-w) # disable all warnings

file(GLOB CLIPSCONTROL_SRC
  ${PROJECT_SOURCE_DIR}/*.cpp
)


## Declare an executable
add_executable(${PROJECT_NAME}
  ${CLIPSCONTROL_SRC}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${CURSES_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

# set_target_properties(clips60
#   PROPERTIES
#   OUTPUT_NAME "lib${PROJECT_NAME}.so"
#   PREFIX ""
#   SUFFIX ""
# )
