# #############################################################################
# CMAKE CONFIGURATION
# #############################################################################
cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING "Final installation location.")
set(CMAKE_BUILD_TYPE_INIT "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/functions.cmake")

project(pointcloudToMESH VERSION 1.0.0)

message("\n" "=========================================")
message("Project: ${PROJECT_NAME} ")
message("=========================================")

# #############################################################################
# PACKAGES
# #############################################################################
message("***********************")
message("PCL PACKAGE")
message("***********************")

find_package(PCL 1.8 REQUIRED QUIET)

if(PCL_FOUND)
    message(STATUS "PCL status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    directory: ${PCL_DIR}")
else()
    message(FATAL_ERROR "   ERROR: PCL minimum required version 1.8. Not found")
endif()

fetch_project(
    NAME argparse
    URL https://github.com/p-ranav/argparse/archive/v2.6.tar.gz
)

# #############################################################################
# SOURCE CODE
# #############################################################################
set(MAIN_SOURCE "src/main.cpp")

# #############################################################################
# EXECUTABLES
# #############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE})

# #############################################################################
# TARGET LIBRARIES
# #############################################################################
target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cpp -mno-avx -mno-avx2)

message("=========================================")
message("Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("=========================================")

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
