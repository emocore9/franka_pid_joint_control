set(CSIR_LIBRARY 
    robot
    threads 
    udp 
)

foreach(library ${CSIR_LIBRARY})
    add_library(${library} ${library}.cc)
endforeach()

target_link_libraries(threads Franka::Franka robot udp)
target_link_libraries(robot Franka::Franka)