INCLUDES = -I$(top_srcdir)/include

EXTRA_DIST = src/hubo_kin.mac src/hubo_kin.f

include_HEADERS = include/hubo_arm.h



lib_LTLIBRARIES = libhuboarm.la
bin_PROGRAMS = harmctrl



libhuboarm_la_SOURCES = src/hubo_kin.f

harmctrl_SOURCES = src/harmctrl.c
harmctrl_LDADD = libhuboarm.la



src/hubo_kin.f: src/hubo_kin.mac
	@echo [maxima start]
	cd src && maxima --very-quiet -b hubo_kin.mac
	@echo [maxima end]
