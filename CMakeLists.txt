
cmake_minimum_required(VERSION 2.8)

set(LIBUCOMM_PARSE_PY ${LIBUCOMM_DIR}/parse.py)
include(${LIBUCOMM_DIR}/cmake/libucomm.cmake)
include_directories(${LIBUCOMM_DIR}/include)

add_subdirectory(host)
add_subdirectory(firmware)
