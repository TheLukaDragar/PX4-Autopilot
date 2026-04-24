px4_add_module(
	MODULE drivers__esc_beep
	MAIN esc_beep
	SRCS
		esc_beep.cpp
	DEPENDS
		px4_work_queue
	)
