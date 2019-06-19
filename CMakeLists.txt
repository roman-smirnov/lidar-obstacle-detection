# TODO: BETTER CMAKE STRUCTURE
cmake_minimum_required(VERSION 3.14.3 FATAL_ERROR)

#file(COPY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# TODO: CHANGE PROJECT NAME TO MEANINGFUL
project(lidar_obstacle_detection VERSION 0.0.1 LANGUAGES CXX)


#if(APPLE)
#    list(APPEND CMAKE_INCLUDE_PATH /opt/local)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# TODO: move render to own module
# TODO: DIVIDE INTO INCLUDE AND SRC
file(GLOB PROJECT_SOURCES src/*.cpp)
file(GLOB PROJECT_HEADERS src/*.h)

# TODO: ONLY FIND WHAT YOU USE
find_package(PCL 1.9.1 CONFIG REQUIRED)  #CONFIG means CMake won't search for a find module


# TODO: target_xxx_yyy(....)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${PROJECT_HEADERS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})




