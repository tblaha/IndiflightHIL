cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 17)

find_package(gz-cmake3 REQUIRED)

# Step 1: change this name (even though also not strictly necessary)
project(MulticopterMotorModelTB)

# Step 2: if you have sources other than *.cc, add them manually
# to the add_library command. Done


find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sim7 REQUIRED)
find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

file(GLOB ModuleSourcesCatchAll "*.cc")
add_library(${CMAKE_PROJECT_NAME} SHARED ${ModuleSourcesCatchAll})
set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_link_directories(${CMAKE_PROJECT_NAME}
  PRIVATE /usr/local/lib/
)
target_link_libraries(${CMAKE_PROJECT_NAME}
  PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sim7::gz-sim7
    gz-msgs9::gz-msgs9)