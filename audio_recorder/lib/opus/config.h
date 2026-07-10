/*
 * config.h — Zephyr-specific opus build configuration.
 * Fixed-point encoder, no intrinsics, no float API.
 */
#ifndef OPUS_CONFIG_H
#define OPUS_CONFIG_H

#define PACKAGE_VERSION "1.5.2"

/* Variable-length arrays — GCC on Cortex-M33 supports them. */
#define VAR_ARRAYS 1

/* lrint / lrintf available in newlib (Zephyr default libc). */
#define HAVE_LRINT  1
#define HAVE_LRINTF 1

/* No VLA debug, no custom modes, no DNN, no DRED. */

#endif /* OPUS_CONFIG_H */
