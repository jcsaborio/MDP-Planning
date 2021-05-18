project(uctMaze)
cmake_minimum_required(VERSION 3.0)

set(SOURCE_FILES
src/maze.cpp
src/UCT.cpp
src/mainUCT.cpp
src/ParserUCT.h
src/Statistic.h
)

set(CMAKE_CXX_FLAGS "-O3")

add_executable(uctMaze ${SOURCE_FILES})
TARGET_LINK_LIBRARIES( uctMaze LINK_PUBLIC )

#set(LIB_DESTINATION "/lib")
#set(BIN_DESTINATION "/bin")

set(dir ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${dir}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${dir}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${dir}/bin")

install(TARGETS uctMaze
  ARCHIVE DESTINATION ${dir}/lib
  LIBRARY DESTINATION ${dir}/lib
  RUNTIME DESTINATION ${dir}/bin
)
