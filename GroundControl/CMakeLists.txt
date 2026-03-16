add_executable(GroundControl main.cpp proxy.cpp)
target_link_libraries(GroundControl PRIVATE SimpleUdp)
target_link_libraries(GroundControl PRIVATE mavlink)
target_link_libraries(GroundControl PRIVATE imgui)
add_dependencies(GroundControl generate_c_headers)
target_compile_options(GroundControl PRIVATE -Wno-address-of-packed-member)
target_include_directories(GroundControl PRIVATE ${CMAKE_BINARY_DIR}/libs/mavlink/include/mavlink/common)

# Copy map image to build directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/images/ülemiste.jpg ${CMAKE_CURRENT_BINARY_DIR}/ülemiste.jpg COPYONLY)
