cmake_minimum_required(VERSION 3.12)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(SVCTest)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

pico_sdk_init()


link_directories($ENV{MICRO_ROS_PICO_SDK_PATH}/libmicroros)

set(EXENAME svctest)
set(SWD_CFG ${CMAKE_CURRENT_SOURCE_DIR}/../swd/raspberrypi5-swd.cfg)
add_custom_target(flash 
	openocd -f ${SWD_CFG} -f target/rp2040.cfg -c \"program ${EXENAME}.elf verify reset exit\"
)

add_executable(${EXENAME} mc_node.c node_helpers.c pico_uart_transport.c)

target_link_libraries(${EXENAME} pico_stdlib microros)

target_include_directories(${EXENAME} 
	PUBLIC $ENV{MICRO_ROS_PICO_SDK_PATH}/libmicroros/include
)

pico_enable_stdio_usb(${EXENAME} 1)
pico_enable_stdio_uart(${EXENAME} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(${EXENAME})

