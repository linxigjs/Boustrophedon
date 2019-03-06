cmake_minimum_required(VERSION 3.9)
project(Boustrophedon)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Boustrophedon Boustrophedon.h main.cpp utils.h InscribedRectFinder.h InscribedRectFinder.cpp)

target_link_libraries(Boustrophedon ${OpenCV_LIBS})

add_executable(mytest test.cpp)
