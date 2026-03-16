cmake_minimum_required(VERSION 3.20)
project(KrattAssignment LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# let's look into lib directory first so that we compile libraries first
set(LIBDIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)
add_subdirectory(${LIBDIR})

# now we can compile sub projects
# please note that we have a separate CMakeLists.txt for each sub project
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Drone)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/GroundControl)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
