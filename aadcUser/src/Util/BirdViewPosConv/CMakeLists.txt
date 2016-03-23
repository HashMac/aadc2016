cmake_minimum_required(VERSION 2.8)
project( BirdViewPosConv )
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( BirdViewPosConv main.cpp BirdViewPosConv.cpp BirdViewPosConv.h )
target_link_libraries( BirdViewPosConv ${OpenCV_LIBS} )
