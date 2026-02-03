COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bmm350_out_of_range_detect.c

API_LOCATION ?= ../..

C_SRCS += \
$(API_LOCATION)/bmm350.c \
$(API_LOCATION)/bmm350_oor.c \
../common/common.c

INCLUDEPATHS += \
$(API_LOCATION) \
../common

TARGET = MCU_APP30

include $(COINES_INSTALL_PATH)/coines.mk