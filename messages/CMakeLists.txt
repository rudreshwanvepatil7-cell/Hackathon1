PROTOBUF_GENERATE_CPP(draco-rpc-msg-src draco-rpc-msg-header draco.proto)
add_library(rpc-draco-msg ${draco-rpc-msg-src} ${draco-rpc-msg-header})
target_include_directories(rpc-draco-msg PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_PYTHON(draco-rpc-msg-py draco.proto)
add_custom_target(draco-rpc-msg-py-target ALL DEPENDS ${draco-rpc-msg-py})

PROTOBUF_GENERATE_CPP(crab-rpc-msg-src crab-rpc-msg-header crab.proto)
add_library(rpc-crab-msg ${crab-rpc-msg-src} ${crab-rpc-msg-header})
target_include_directories(rpc-crab-msg PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_PYTHON(crab-rpc-msg-py crab.proto)
add_custom_target(crab-rpc-msg-py-target ALL DEPENDS ${crab-rpc-msg-py})

install(TARGETS rpc-draco-msg DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${PROJECT_SOURCE_DIR}/build/messages/draco.pb.h DESTINATION
    "${INSTALL_INCLUDE_DIR}/messages")
install(TARGETS rpc-crab-msg DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${PROJECT_SOURCE_DIR}/build/messages/crab.pb.h DESTINATION
    "${INSTALL_INCLUDE_DIR}/messages")
