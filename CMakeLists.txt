cmake_minimum_required(VERSION 3.28)
project(lidar_sim LANGUAGES CXX)

# set up main executable
add_executable(${PROJECT_NAME} "src/main.cpp")

# set up FetchContent for deps
include(FetchContent)
set(FETCHCONTENT_QUIET OFF) # enable git output for FetchContent steps
set(FETCHCONTENT_UPDATES_DISCONNECTED ON) # speed up consecutive config runs

# set up dependencies
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW) # disallow option() from overwriting set()
seT(CMAKE_CUDA_ARCHITECTURES native)
set(ASSIMP_BUILD_TESTS OFF)
set(BUILD_TESTS OFF)
set(BUILD_TOOLS OFF)
FetchContent_Declare(assimp
    GIT_REPOSITORY "https://github.com/assimp/assimp.git"
    GIT_TAG "v5.4.3"
    GIT_SHALLOW ON
    OVERRIDE_FIND_PACKAGE
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_Declare(rmagine
    GIT_REPOSITORY "https://github.com/uos/rmagine.git"
    GIT_TAG "main"
    GIT_SHALLOW ON
    OVERRIDE_FIND_PACKAGE
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_Declare(sponza
    GIT_REPOSITORY "https://github.com/M2-TE/Sponza.git"
    GIT_TAG "s0.01"
    GIT_SHALLOW ON
    SOURCE_DIR "${CMAKE_BINARY_DIR}/sponza")
FetchContent_MakeAvailable(assimp rmagine sponza)
target_link_libraries(${PROJECT_NAME} PRIVATE rmagine::core rmagine::embree)