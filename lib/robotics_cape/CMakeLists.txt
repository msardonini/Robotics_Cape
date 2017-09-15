
cmake_minimum_required(VERSION 2.8)

project(ROBOTICSCAPE)

set(ROBOTICSCAPE_DIR lib/robotics_cape/)

set(SRCS
	${ROBOTICSCAPE_DIR}pru_handler_client.c
	${ROBOTICSCAPE_DIR}roboticscape.c
	${ROBOTICSCAPE_DIR}bmp280/rc_bmp280.c
	${ROBOTICSCAPE_DIR}gpio/rc_buttons.c
	${ROBOTICSCAPE_DIR}gpio/rc_gpio.c
	${ROBOTICSCAPE_DIR}gpio/rc_gpio_setup.c
	${ROBOTICSCAPE_DIR}math/rc_filter.c
	${ROBOTICSCAPE_DIR}math/rc_linear_algebra.c
	${ROBOTICSCAPE_DIR}math/rc_matrix.c
	${ROBOTICSCAPE_DIR}math/rc_neon_functions.c
	${ROBOTICSCAPE_DIR}math/rc_polynomial.c
	${ROBOTICSCAPE_DIR}math/rc_quaternion.c
	${ROBOTICSCAPE_DIR}math/rc_ring_buffer.c
	${ROBOTICSCAPE_DIR}math/rc_vector.c
	${ROBOTICSCAPE_DIR}mmap/rc_mmap_gpio_adc.c
	${ROBOTICSCAPE_DIR}mmap/rc_mmap_pwmss.c
	${ROBOTICSCAPE_DIR}mpu9250/rc_mpu9250.c
	${ROBOTICSCAPE_DIR}other/rc_bb_model.c
	${ROBOTICSCAPE_DIR}other/rc_cpu_freq.c
	${ROBOTICSCAPE_DIR}other/rc_dsm.c
	${ROBOTICSCAPE_DIR}other/rc_other.c
	${ROBOTICSCAPE_DIR}other/rc_pinmux.c
	${ROBOTICSCAPE_DIR}other/rc_pru.c
	${ROBOTICSCAPE_DIR}other/rc_time.c
	${ROBOTICSCAPE_DIR}pwm/rc_motors.c
	${ROBOTICSCAPE_DIR}pwm/rc_pwm.c
	${ROBOTICSCAPE_DIR}serial_ports/rc_i2c.c
	${ROBOTICSCAPE_DIR}serial_ports/rc_spi.c
	${ROBOTICSCAPE_DIR}serial_ports/rc_uart.c
	)

#add_definitions(-DPOSIX_SHARED)

add_library(roboticscape SHARED ${SRCS})
#install(TARGETS roboticscape DESTINATION "/usr/lib/")
