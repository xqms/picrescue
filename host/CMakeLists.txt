
libucomm_wrap_msg(PROTO_MSG ../proto_picrescue.uc)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(picrescue_host
	picrescue.cpp
	${PROTO_MSG}
)
