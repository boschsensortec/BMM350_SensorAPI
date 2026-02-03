COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bmm350_out_of_range_detect.c

API_LOCATION ?= ../..
COMMON_LOCATION ?= ..

C_SRCS += \
$(API_LOCATION)/bmm350.c \
$(API_LOCATION)/bmm350_oor.c \
$(COMMON_LOCATION)/common/common.c


INCLUDEPATHS += \
$(API_LOCATION) \
$(COMMON_LOCATION)/common

ifndef TARGET
$(error TARGET is not defined; please specify a target)
endif
INVALID_TARGET = PC
$(if $(filter $(TARGET),PC), $(error TARGET has an invalid value '$(TARGET)'; 'PC' is not a valid target. Please use a MCU target (eg. MCU_APP30).), )

include $(COINES_INSTALL_PATH)/coines.mk