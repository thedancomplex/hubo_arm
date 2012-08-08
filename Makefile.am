INCLUDES = -I$(top_srcdir)/include

EXTRA_DIST = src/hubo_kin.mac src/hubo_kin.f

include_HEADERS = include/hubo_arm.h

src/hubo_kin.f: src/hubo_kin.mac
	@echo [maxima start]
	cd src && maxima --very-quiet -b hubo_kin.mac
	@echo [maxima end]

lib_LTLIBRARIES = libhuboarm.la

libhuboarm_la_SOURCES =                  \
	src/hubo_kin.f
