project(maze)
cmake_minimum_required(VERSION 3.0)

set(SOURCE_FILES
src/maze.cpp
src/vi.cpp
src/main.cpp
src/Parser.h
)

set(CMAKE_CXX_FLAGS "-O3")

add_executable(maze ${SOURCE_FILES})
TARGET_LINK_LIBRARIES( maze LINK_PUBLIC )

#set(LIB_DESTINATION "/lib")
#set(BIN_DESTINATION "/bin")

set(dir ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${dir}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${dir}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${dir}/bin")

install(TARGETS maze
  ARCHIVE DESTINATION ${dir}/lib
  LIBRARY DESTINATION ${dir}/lib
  RUNTIME DESTINATION ${dir}/bin
)
