# SPDX-License-Identifier: MIT
# CMake glue for the openbricks user_c_module.
#
# Referenced from boards/openbricks_esp32/mpconfigboard.cmake via
# ``USER_C_MODULES=``. For a different MicroPython port, either import
# this file the same way or use ``micropython.mk`` for Makefile-based
# ports (RP2040, stm32, etc).

add_library(usermod_openbricks INTERFACE)

target_sources(usermod_openbricks INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/motor_process.c
    ${CMAKE_CURRENT_LIST_DIR}/trajectory.c
    ${CMAKE_CURRENT_LIST_DIR}/observer.c
    ${CMAKE_CURRENT_LIST_DIR}/servo.c
    ${CMAKE_CURRENT_LIST_DIR}/drivebase.c
    ${CMAKE_CURRENT_LIST_DIR}/pcnt_encoder.c
    ${CMAKE_CURRENT_LIST_DIR}/openbricks_module.c
)

target_include_directories(usermod_openbricks INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_openbricks)
