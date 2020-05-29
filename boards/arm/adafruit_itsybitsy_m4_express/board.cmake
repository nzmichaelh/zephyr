# Copyright (c) 2020 Google LLC.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(bossac "--offset=0x4000")

#include(${ZEPHYR_BASE}/boards/common/bossac.board.cmake)

board_runner_args(openocd --cmd-pre-load "reset halt" --cmd-pre-load "atsame5 bootloader 0" --cmd-pre-load "reset halt" --cmd-pre-load "atsame5 chip-erase" --cmd-pre-load "sleep 5000")
set(OPENOCD_FLASH "flash write_image")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
