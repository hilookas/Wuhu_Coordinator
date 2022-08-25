cmake_minimum_required( VERSION 3.4...3.18 )
project( main )

find_package( OpenCV 4.5.5 EXACT REQUIRED PATHS /opt/opencv ) # Ref: https://stackoverflow.com/questions/49816206/cmake-find-package-specify-path
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
set( CMAKE_BUILD_TYPE Debug )

aux_source_directory( ./src SRC_LIST )
add_executable( main ${SRC_LIST} )
target_link_libraries( main ${OpenCV_LIBS} )