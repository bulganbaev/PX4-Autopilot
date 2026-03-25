px4_add_module(
	MODULE modules__matek_cam_switch
	MAIN matek_cam_switch
	STACK_MAIN 2200
	SRCS
		matek_cam_switch.cpp
)
