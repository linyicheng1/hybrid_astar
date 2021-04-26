cmake_minimum_required(VERSION 3.8)
project(hybrid_astar)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
include_directories(
        inc
        ${OpenCV_INCLUDE_DIRS}
)

add_library(
        planner
        src/planner.cpp
        src/node.cpp
        src/hybridAStar.cpp
        src/CollisionDetection.cpp
        src/ReedsSheppPath.cpp
)

add_executable(hybrid_astar main.cpp)
target_link_libraries(hybrid_astar planner ${OpenCV_LIBS})

add_executable(test_a_star test/test_A_star.cpp)
target_link_libraries(test_a_star planner ${OpenCV_LIBS})

add_executable(test_map test/test_map.cpp)
target_link_libraries(test_map planner ${OpenCV_LIBS})

add_executable(test_RS test/test_ReedsSheppPath.cpp)
target_link_libraries(test_RS planner ${OpenCV_LIBS})

add_executable(test_hybrid_aStar test/test_hybrid_aStar.cpp)
target_link_libraries(test_hybrid_aStar planner ${OpenCV_LIBS})

