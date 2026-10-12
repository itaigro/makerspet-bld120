cmake_minimum_required(VERSION 3.8)
project(bld120_bringup)

find_package(ament_cmake REQUIRED)

# Install every resource directory that exists; directories are added as the
# milestones introduce launch files and configuration.
foreach(dir launch config)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
