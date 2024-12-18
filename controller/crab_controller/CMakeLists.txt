file(GLOB_RECURSE sources "*.cpp" "*.c")
file(GLOB controller_headers "*.hpp")

add_library(rpc-crab-controller SHARED ${sources} ${controller_headers})
target_link_libraries(rpc-crab-controller PUBLIC rpc-pin-robot-system
                                           PUBLIC rpc-wbc
                                           PUBLIC rpc-util
                                           )


install(TARGETS rpc-crab-controller DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${controller_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/crab_controller")
