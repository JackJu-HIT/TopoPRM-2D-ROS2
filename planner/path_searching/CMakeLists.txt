project(path_searching)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++17 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
include_directories(include)
include_directories(../GridMap2D/include)


add_library( path_searching STATIC
    src/kinodynamic_astar.cpp
    src/topo_prm.cpp
    src/raycast.cpp
    )
