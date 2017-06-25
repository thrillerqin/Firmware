px4_add_module(
	MODULE modules__tianteng_atmos_detect
	MAIN tianteng_atmos_detect
	STACK_MAIN 2000
	SRCS
		tianteng_atmos_detect.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 