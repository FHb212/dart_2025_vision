cmake_minimum_required(VERSION 3.8)
project(dart_transporter)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build SDK as subdirectory (produces transporter_sdk library)
add_subdirectory(include/sdk)

# Include directories for public headers
include_directories(
	include
)

# Node executable (minimal stub for now)
add_executable(usb_node src/usb_node.cpp)
ament_target_dependencies(usb_node rclcpp std_msgs)
target_link_libraries(usb_node transporter_sdk)
target_include_directories(usb_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

install(TARGETS usb_node
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
	DESTINATION include
)

install(DIRECTORY config/
	DESTINATION share/${PROJECT_NAME}
)

ament_package()