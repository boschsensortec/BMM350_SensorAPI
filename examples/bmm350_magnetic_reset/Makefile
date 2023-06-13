COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bmm350_magnetic_reset.c

API_LOCATION ?= ../..

C_SRCS += \
$(API_LOCATION)/bmm350.c \
../common/common.c

INCLUDEPATHS += \
$(API_LOCATION) \
../common

COINES_BACKEND=COINES_BRIDGE

include $(COINES_INSTALL_PATH)/coines.mk
