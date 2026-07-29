# Include from the ModusToolbox CM55 project Makefile before make/start.mk.
HBALL_EDGETALK_ROOT?=../../firmware/edgetalk

CORE=CM55
CORE_NAME=CM55_0
COMPONENTS+=FREERTOS
VFP_SELECT=hardfp
VFP_SELECT_PRECISION=doublefp
MVE_SELECT=NO_MVE

DEFINES+=HBALL_M55_SHADOW_ONLY=1
INCLUDES+=$(HBALL_EDGETALK_ROOT)/include
INCLUDES+=$(HBALL_EDGETALK_ROOT)/freertos

SOURCES+=$(HBALL_EDGETALK_ROOT)/freertos/main_cm55.c
SOURCES+=$(HBALL_EDGETALK_ROOT)/freertos/hball_m55_shadow_task.c
SOURCES+=$(HBALL_EDGETALK_ROOT)/src/hball_control_pipeline.c
SOURCES+=$(HBALL_EDGETALK_ROOT)/src/hball_dualcore_ipc.c
SOURCES+=$(HBALL_EDGETALK_ROOT)/src/hball_dualcore_platform.c
SOURCES+=$(HBALL_EDGETALK_ROOT)/src/hball_lqg.c
SOURCES+=$(HBALL_EDGETALK_ROOT)/src/hball_m55_ipc.c

LDLIBS+=m
