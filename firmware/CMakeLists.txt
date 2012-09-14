
cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)
set(CMAKE_LINKER avr-ld)

set(CMAKE_CXX_FLAGS "-mmcu=atmega32 -O2 -Wall")

add_definitions(-DF_CPU=16000000UL)

libucomm_wrap_msg(PROTO_MSG ../proto_picrescue.uc)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(picrescue
	main.cpp
	${PROTO_MSG}
)

add_custom_command(
	OUTPUT picrescue.hex
	DEPENDS picrescue
	COMMAND avr-objcopy -O ihex picrescue picrescue.hex
)

add_custom_target(writeflash
	DEPENDS picrescue.hex
	COMMAND sudo avrdude -c usbasp -p atmega32 -Uflash:w:picrescue.hex
)
