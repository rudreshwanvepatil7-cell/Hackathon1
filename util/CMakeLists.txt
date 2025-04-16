FILE(GLOB_RECURSE sources RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")
FILE(GLOB_RECURSE headers RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.hpp")

list(FILTER sources EXCLUDE REGEX "^mujoco_utils/")

add_library(rpc-util ${sources} ${headers})
target_link_libraries(rpc-util PUBLIC rpc-yaml)

install(TARGETS rpc-util DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/util")

add_subdirectory(mujoco_utils)