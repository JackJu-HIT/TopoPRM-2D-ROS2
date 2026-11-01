
include_directories(include)
include_directories(../path_searching/include)
include_directories(./include/plan_manage)

add_library(    kinodynamic_planner  STATIC
                src/planner_interface.cpp
)

target_link_libraries(kinodynamic_planner
                       path_searching
                       local_map )
