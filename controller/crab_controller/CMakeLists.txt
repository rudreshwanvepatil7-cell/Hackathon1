file(GLOB_RECURSE sources "*.cpp" "*.c")
file(GLOB controller_headers "*.hpp")
file(GLOB state_machines_headers "crab_state_machines/*.hpp")
file(GLOB crab_task_headers "crab_task/*.hpp" "crab_task/*.h")

list(REMOVE_ITEM sources "${CMAKE_CURRENT_SOURCE_DIR}/crab_data_manager.cpp")
list(REMOVE_ITEM controller_headers "${CMAKE_CURRENT_SOURCE_DIR}/crab_data_manager.hpp")

if(BUILD_WITH_ZMQ_PROTOBUF)
    list(APPEND sources "${CMAKE_CURRENT_SOURCE_DIR}/crab_data_manager.cpp")
    list(APPEND controller_headers "${CMAKE_CURRENT_SOURCE_DIR}/crab_data_manager.hpp")
endif()

add_library(rpc-crab-controller SHARED ${sources} ${controller_headers}
        ${state_machines_headers} ${crab_task_headers})

target_link_libraries(rpc-crab-controller PUBLIC rpc-pin-robot-system
                                          PUBLIC rpc-wbc
                                          PUBLIC rpc-util
                                          PUBLIC rpc-dcm-planner
                                          PUBLIC rpc-filter
                                          PUBLIC rpc-state-estimator
                                          PUBLIC rpc-model
                                            )

if(BUILD_WITH_ZMQ_PROTOBUF)
    target_link_libraries(rpc-crab-controller PUBLIC ${Protobuf_LIBRARIES}
            PUBLIC ${ZMQ_LIBRARIES}
            PUBLIC rpc-crab-msg)
endif()

if(BUILD_WITH_MATLOGGER)
    target_link_libraries(rpc-crab-controller PUBLIC matlogger2::matlogger2)
endif()

if(BUILD_WITH_FOXGLOVE)
    target_link_libraries(rpc-crab-controller PUBLIC foxglove-parameter-subscriber)
endif()


install(TARGETS rpc-crab-controller DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${controller_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/crab_controller")
install(FILES ${state_machines_headers} DESTINATION
        "${INSTALL_INCLUDE_DIR}/controller/crab_controller/crab_state_machines")
install(FILES ${crab_task_headers} DESTINATION
        "${INSTALL_INCLUDE_DIR}/controller/crab_controller/crab_task")
