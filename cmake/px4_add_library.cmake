#=============================================================================
#
#	px4_add_library
#
#	Like add_library but with PX4 platform dependencies
#
function(px4_add_library target)
	add_library(${target} ${ARGN})

	target_compile_definitions(${target} PRIVATE MODULE_NAME="${target}")

	install(TARGETS ${target}
		DESTINATION /usr/lib
		)


	# # all PX4 libraries have access to parameters and uORB
	# add_dependencies(${target} uorb_headers)
	# target_link_libraries(${target} PRIVATE prebuild_targets parameters_interface uorb_msgs)

	# # TODO: move to platform layer
	# if ("${PX4_PLATFORM}" MATCHES "nuttx")
	# 	target_link_libraries(${target} PRIVATE m nuttx_c)
	# endif()

	# # Pass variable to the parent px4_add_module.
	# set(_no_optimization_for_target ${_no_optimization_for_target} PARENT_SCOPE)

	# set_property(GLOBAL APPEND PROPERTY PX4_LIBRARIES ${target})
	# set_property(GLOBAL APPEND PROPERTY PX4_MODULE_PATHS ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()