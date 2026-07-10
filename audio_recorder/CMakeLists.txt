# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(audio_recorder)

target_sources(app PRIVATE src/main.c src/ble_audio.c)

# Opus encoder library
add_subdirectory(lib/opus)
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/opus-1.5.2/include
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/opus
)
target_link_libraries(app PRIVATE opus)
