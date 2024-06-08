cmake_minimum_required(VERSION 3.0.2)
set(PARENT_PROJECT_NAME "${PROJECT_NAME}")
project(clipscontrol)

if (PARENT_PROJECT_NAME)
  add_compile_definitions(ROSPACKAGE=${PARENT_PROJECT_NAME})
  add_compile_definitions(QUERYKDB_H="${PARENT_PROJECT_NAME}/QueryKDB.h")
endif ()


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
