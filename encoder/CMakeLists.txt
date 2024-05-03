cmake_minimum_required(VERSION 3.12)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(QuadratureDecoderTest)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(EXENAME qe_test)

set(SWD_CFG ../swd/raspberrypi5-swd.cfg)
add_custom_target(flash 
	openocd -f ${SWD_CFG} -f target/rp2040.cfg -c \"program ${EXENAME}.elf verify reset exit\"
)


add_executable(${EXENAME})
pico_generate_pio_header(${EXENAME} ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)
target_sources(${EXENAME} PUBLIC quadrature_encoder_test.c PUBLIC quadrature_encoder.c) 

pico_enable_stdio_usb(${EXENAME} 1)
pico_enable_stdio_uart(${EXENAME} 0)
pico_add_extra_outputs(${EXENAME})
target_link_libraries(${EXENAME} 
	pico_stdlib 
	hardware_pio
	)

