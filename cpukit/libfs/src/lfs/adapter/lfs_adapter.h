/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup Flash
 *
 * @brief Generic Flash API
 */

/*
 * Copyright (C) 2024 Bernd Moessner
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LFS_ADAPTER_H
#define LFS_ADAPTER_H

#include <rtems.h>
#include <rtems/thread.h>
#include <dev/flash/flashdev.h>
#include <lfs.h>

#ifdef __cplusplus
extern "C"
{
#endif



/**
 * @brief Context handed over to LittleFS
 */
typedef struct {
  /**
   * @brief Pointer to the flash device on which the filesystem resides.
   * The flash device must have been fully initialized by the application
   */
  rtems_flashdev * flashdev;

  /**
   * @brief File Descriptor
   */
  int32_t flashdev_fd;

  /**
   * @brief Pointer to the flash partition. The partion must not neccessarily
   * be created within flashdev. It servers only to tell the LittleFS adapter
   * code about the flash geometry (offset, size) 
   */
  rtems_flashdev_partition partition;

  /**
   * @brief Partition index to be used as returned by the flashdev
   * partition create command, or -1 for signaling that no 
   * partition is to be used
   */
  int32_t flashdev_partition_idx;

  struct lfs_config lfs_config;
} rtems_lfs_context_t;

/**
 * @brief Flash device.
 */
typedef struct {
  rtems_recursive_mutex s_mutex;
  lfs_file_t lfs_file;
  lfs_t lfs;
  rtems_lfs_context_t ctx;
} rtems_lfs_fs_info_t;


/**
 * @brief Translate LittleFS error code to RTEMS 
 *
 * Will set errno and return -1 in case an error was detected
 *
 * @param[in] error The return of an LittleFS function
 *
 * @retval -1 Failed to set up flash device.
 * @retval >=0 Successful
 */
int lfs_error_to_errno(int error);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // LFS_ADAPTER_H
