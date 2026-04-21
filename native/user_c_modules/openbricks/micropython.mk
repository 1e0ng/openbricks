# SPDX-License-Identifier: MIT
# Makefile glue for the openbricks user_c_module. Used by MicroPython
# ports that build with make (unix, stm32). The CMake port (ESP-IDF,
# rp2040) uses the companion ``micropython.cmake`` instead.

USERMOD_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

SRC_USERMOD += $(USERMOD_DIR)/motor_process.c
SRC_USERMOD += $(USERMOD_DIR)/trajectory.c
SRC_USERMOD += $(USERMOD_DIR)/observer.c
SRC_USERMOD += $(USERMOD_DIR)/servo.c
SRC_USERMOD += $(USERMOD_DIR)/drivebase.c
SRC_USERMOD += $(USERMOD_DIR)/pcnt_encoder.c
SRC_USERMOD += $(USERMOD_DIR)/encoder.c
SRC_USERMOD += $(USERMOD_DIR)/bno055.c
SRC_USERMOD += $(USERMOD_DIR)/openbricks_module.c
CFLAGS_USERMOD += -I$(USERMOD_DIR)
