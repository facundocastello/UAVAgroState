cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( main )
find_package( OpenCV REQUIRED )
add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} "/usr/local/lib/libhpdf.so")

