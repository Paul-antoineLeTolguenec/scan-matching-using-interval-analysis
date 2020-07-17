cmake_minimum_required(VERSION 3.0.2)
project(Scan_matching LANGUAGES CXX)

# Compilation options
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-O3 -DNDEBUG) # comment for debug mode

# Dependencies
include(FindPkgConfig)
pkg_search_module(TUBEX REQUIRED tubex)
message(STATUS "Found Tubex version ${TUBEX_VERSION}")

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-Wno-deprecated" COMPILER_SUPPORTS_WNO_DEPRECATED)

# Compilation
add_executable(${PROJECT_NAME} src/main.cpp src/OpHull.cpp)
target_compile_options(${PROJECT_NAME} PUBLIC ${TUBEX_CFLAGS})
target_include_directories(${PROJECT_NAME} PUBLIC ${TUBEX_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${TUBEX_LDFLAGS})

