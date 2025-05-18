cmake_minimum_required(VERSION 3.13)

# find opencv
find_package(OpenCV REQUIRED)

add_subdirectory(pfr)

add_library(${PROJECT_NAME}_libs INTERFACE)
target_link_libraries(${PROJECT_NAME}_libs INTERFACE
    ${OpenCV_LIBS}
    Boost::pfr
)
