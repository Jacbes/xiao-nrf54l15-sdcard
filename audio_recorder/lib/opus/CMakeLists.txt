# Opus encoder library for Zephyr.
# Fixed-point, no intrinsics.

set(OPUS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../opus-1.5.2)

file(GLOB_RECURSE OPUS_ALL_SRCS
  ${OPUS_ROOT}/src/*.c
  ${OPUS_ROOT}/celt/*.c
  ${OPUS_ROOT}/silk/*.c
)

# Exclude platform-specific (x86, ARM NEON/ASM), float, tests, demos
file(GLOB OPUS_X86 ${OPUS_ROOT}/celt/x86/*.c ${OPUS_ROOT}/silk/x86/*.c ${OPUS_ROOT}/silk/fixed/x86/*.c ${OPUS_ROOT}/silk/float/x86/*.c)
file(GLOB OPUS_ARM  ${OPUS_ROOT}/celt/arm/*.c ${OPUS_ROOT}/celt/arm/*.s ${OPUS_ROOT}/silk/arm/*.c ${OPUS_ROOT}/silk/fixed/arm/*.c ${OPUS_ROOT}/silk/float/arm/*.c)
file(GLOB OPUS_FLOAT ${OPUS_ROOT}/silk/float/*.c)
file(GLOB OPUS_TESTS ${OPUS_ROOT}/silk/tests/*.c ${OPUS_ROOT}/celt/tests/*.c)
file(GLOB OPUS_DEMOS ${OPUS_ROOT}/src/opus_demo.c ${OPUS_ROOT}/src/opus_compare.c ${OPUS_ROOT}/src/opus_custom_demo.c)

list(REMOVE_ITEM OPUS_ALL_SRCS ${OPUS_X86} ${OPUS_ARM} ${OPUS_FLOAT} ${OPUS_TESTS} ${OPUS_DEMOS})

zephyr_library_named(opus)
zephyr_library_sources(${OPUS_ALL_SRCS})

zephyr_library_include_directories(
  ${OPUS_ROOT}/include
  ${OPUS_ROOT}/silk
  ${OPUS_ROOT}/silk/fixed
  ${OPUS_ROOT}/celt
  ${OPUS_ROOT}/src
  ${CMAKE_CURRENT_SOURCE_DIR}
)

zephyr_library_compile_definitions(
  OPUS_BUILD
  FIXED_POINT=1
  DISABLE_FLOAT_API
  VAR_ARRAYS=1
  HAVE_LRINT
  HAVE_LRINTF
  HAVE_CONFIG_H
  OPUS_ARM_PRESUME_NEON=0
  OPUS_ARM_MAY_HAVE_NEON=0
)

zephyr_library_compile_options(
  -Wno-unused-parameter
  -Wno-sign-compare
  -Wno-maybe-uninitialized
  -Wno-double-promotion
  -Wno-stringop-overflow
)
