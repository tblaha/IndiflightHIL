cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 17)

find_package(gz-cmake3 REQUIRED)

# Step 1: change this name (even though also not strictly necessary)
project(Indiflight)

# Step 2: if you have sources other than *.cc, add them manually
# to the add_library command. Done


find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sim7 REQUIRED)
find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# needed for asio comms
find_package(Boost REQUIRED COMPONENTS system)

# ----- python generation of pi protocol headers -----
set(PI_GEN_DIR ${CMAKE_BINARY_DIR}/pi-protocol)
include_directories(${PI_GEN_DIR})

# find Py3, setting the Python3_EXECUTATBLE CMAKE variable
find_package(Python3 COMPONENTS Interpreter)
#message(${Python3_EXECUTABLE})

set(PI_SRC_DIR ${CMAKE_SOURCE_DIR}/pi-protocol)
set(PI_CONFIG_FILENAME config.yaml)
add_custom_command(
    OUTPUT
        ${PI_GEN_DIR}/pi-protocol.h
        ${PI_GEN_DIR}/pi-messages.h
        ${PI_GEN_DIR}/pi-messages.c
    DEPENDS
        ${PI_SRC_DIR}/${PI_CONFIG_FILENAME}
        ${PI_SRC_DIR}/msgs/*.yaml
        ${PI_SRC_DIR}/python/*.py
        ${PI_SRC_DIR}/templates/*.j2
    COMMAND mkdir -p ${PI_GEN_DIR} && ${Python3_EXECUTABLE}
        ${PI_SRC_DIR}/python/generate.py
        ${PI_CONFIG_FILENAME}
        --output-dir ${PI_GEN_DIR})
configure_file(${PI_SRC_DIR}/src/pi-protocol.c ${PI_GEN_DIR}/pi-protocol.c COPYONLY)

file(GLOB ModuleSourcesCatchAll "*.cc")
add_library(${CMAKE_PROJECT_NAME} SHARED ${ModuleSourcesCatchAll} ${PI_GEN_DIR}/pi-protocol.c ${PI_GEN_DIR}/pi-messages.c)
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