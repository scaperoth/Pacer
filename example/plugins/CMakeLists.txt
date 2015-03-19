add_library(drive-robot drive-robot.cpp)
target_link_libraries(drive-robot ${REQLIBS})

add_library(ik-feet ik_feet.cpp)
target_link_libraries(ik-feet ${REQLIBS})

add_library(gait-planner gait-planner.cpp)
target_link_libraries(gait-planner ${REQLIBS})

add_library(joint-PID-controller MODULE joint-PID-controller.cpp)
target_link_libraries(joint-PID-controller ${REQLIBS})

add_library(stand-trajectory MODULE stand.cpp)
target_link_libraries(stand-trajectory ${REQLIBS})