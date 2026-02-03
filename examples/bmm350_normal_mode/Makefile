COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bmm350_normal_mode.c

API_LOCATION ?= ../..
COMMON_LOCATION ?= ..

C_SRCS += \
$(API_LOCATION)/bmm350.c \
$(COMMON_LOCATION)/common/common.c

INCLUDEPATHS += \
$(API_LOCATION) \
$(COMMON_LOCATION)/common

COINES_BACKEND=COINES_BRIDGE

include $(COINES_INSTALL_PATH)/coines.mk