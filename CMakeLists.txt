# Minimum version of CMake required to build this project
cmake_minimum_required(VERSION 3.5)

# Name of the project
project(friction_estimation)

set(CMAKE_INCLUDE_CURRENT_DIR   ON)
set(CMAKE_VERBOSE_MAKEFILE      ON)
set(CMAKE_COLOR_MAKEFILE        ON)
<<<<<<< HEAD

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
=======
set(CMAKE_BUILD_TYPE Release)
>>>>>>> 800e6a002df8bbd21f9696354c1274d95e85ea6c

# Activate C++ 11
set (CMAKE_CXX_STANDARD 14)

#how to print
message("----: ${PROJECT_SOURCE_DIR}")

include_directories(${PROJECT_SOURCE_DIR}/thirdParty/cxxopts/)

add_compile_options(-Wall)
add_compile_options(-Wno-reorder)

if(UNIX)
  add_definitions(-D_OS_UNIX)
endif()

 # Setup Kortex Api Path

set(KORTEX_DIR "${PROJECT_SOURCE_DIR}/kortex_api/")
message("----: ${KORTEX_DIR}")

link_libraries(${KORTEX_DIR}lib/release/libKortexApiCpp.a)

# Add Include Directories
include_directories(${KORTEX_DIR}/include)
include_directories(${KORTEX_DIR}/include/client)
include_directories(${KORTEX_DIR}/include/common)
include_directories(${KORTEX_DIR}/include/messages)
include_directories(${KORTEX_DIR}/include/client_stubs)

# user code
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

link_libraries(pthread)
link_libraries(${KORTEX_DIR}lib/release/libKortexApiCpp.a)
add_executable(main
  src/main_kinova.cpp
  src/friction_observer.cpp 
)

add_executable(main
    src/main_kinova.cpp
    src/friction_observer.cpp
)
