cmake_minimum_required(VERSION 3.0.2)
project(clips)

add_definitions(-w) # disable all warnings

file(GLOB CLIPS_SRC
  ${PROJECT_SOURCE_DIR}/src/*.c
)

## Declare a C library
add_library(clips60
  ${CLIPS_SRC}
)

target_include_directories(clips60
  PRIVATE
  ${PROJECT_SOURCE_DIR}/include/clips
)

target_include_directories(clips60
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

# set_target_properties(clips60
#   PROPERTIES
#   OUTPUT_NAME "lib${PROJECT_NAME}.so"
#   PREFIX ""
#   SUFFIX ""
# )
