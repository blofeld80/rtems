/* SPDX-License-Identifier: BSD-2-Clause */

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

#ifndef RTEMS_LFS_H
#define RTEMS_LFS_H

//#include <lfs.h>
#include <rtems/fs.h>
//#include <sys/param.h>
//#include <sys/ioccom.h>
#include <dev/flash/flashdev.h>



#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct rtems_lfs_mount_data rtems_lfs_mount_data;

/**
 * @defgroup LitteFS File System Support
 *
 * @ingroup FileSystemTypesAndMount
 *
 * @brief Mount options for LittleFS
 *
 * The application must provide flash device geometry information and flash
 * device operations in the flash control structure
 * @ref rtems_lfs_mount_data.
 *
 * The application must enable LittleFS support with 
 * rtems_filesystem_register() or CONFIGURE_FILESYSTEM_LITTLEFS via 
 * <rtems/confdefs.h>.
 * 
 * @{
 */




/**
 * @brief LFS mount data.
 *
 * For LFS the mandatory mount data.
 */
struct rtems_lfs_mount_data {

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

  /**
   * @brief Flag to indicate if formating is allowed. It is required
   * to format the file system once.
   */
  bool formating_allowed;

  /**
   * @brief Information returned about the page. Including the
   * base offset and size of page.
   */
  void * lfs_config;
};




/**
 * @brief Initialization handler of the LittleFS file system.
 *
 * @param[in, out] mt_entry The mount table entry.
 * @param[in] data The mount options are mandatory for LittleFS and data must
 * point to a valid @ref rtems_lfs_mount_data structure used for this file
 * system instance.
 *
 * @retval 0 Successful operation.
 * @retval -1 An error occurred.  The @c errno indicates the error.
 *
 * @see mount().
 */
int rtems_lfs_initialize(
  rtems_filesystem_mount_table_entry_t *mt_entry,
  void *data
);



/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RTEMS_LFS_H */
