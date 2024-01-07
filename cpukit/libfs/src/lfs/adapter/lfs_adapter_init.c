/* SPDX-License-Identifier: BSD-2-Clause */

/**
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

#include <rtems/thread.h>
#include <rtems.h>
#include <rtems/bspIo.h>
#include <string.h>
#include <assert.h>
#include <rtems/libio.h>
#include <rtems/libio_.h>
#include <rtems/sysinit.h>
#include <rtems/lfs.h>
#include <errno.h>

#include "lfs_adapter.h"
#include "lfs_flashdev.h"



int lfs_error_to_errno(int error)
{
  if (error >= 0) {
    return error;
  }

  switch (error) {
    /* Error during device operation */
    default:
    case LFS_ERR_IO:
      rtems_set_errno_and_return_minus_one(EIO);
    /* Corrupted */
    case LFS_ERR_CORRUPT:
      rtems_set_errno_and_return_minus_one(EFAULT);
    /* No directory entry */
    case LFS_ERR_NOENT:
      rtems_set_errno_and_return_minus_one(ENOENT);
    /* Entry already exists */
    case LFS_ERR_EXIST:
      rtems_set_errno_and_return_minus_one(EEXIST);
    /* Entry is not a dir */
    case LFS_ERR_NOTDIR:
      rtems_set_errno_and_return_minus_one(ENOTDIR);
    /* Entry is a dir */
    case LFS_ERR_ISDIR:
      rtems_set_errno_and_return_minus_one(EISDIR);
    /* Dir is not empty */
    case LFS_ERR_NOTEMPTY:
      rtems_set_errno_and_return_minus_one(ENOTEMPTY);
    /* Bad file number */
    case LFS_ERR_BADF:
      rtems_set_errno_and_return_minus_one(EBADF);
    /* File too large */
    case LFS_ERR_FBIG:
      rtems_set_errno_and_return_minus_one(EFBIG);
    /* Invalid parameter */
    case LFS_ERR_INVAL:
      rtems_set_errno_and_return_minus_one(EINVAL);
    /* No space left on device */
    case LFS_ERR_NOSPC:
      rtems_set_errno_and_return_minus_one(ENOSPC);
    /* No more memory available */
    case LFS_ERR_NOMEM:
      rtems_set_errno_and_return_minus_one(ENOMEM);
  }
}

extern const rtems_filesystem_file_handlers_r  rtems_lfs_file_handlers;
extern const rtems_filesystem_file_handlers_r  rtems_lfs_dir_handlers;
extern const rtems_filesystem_operations_table rtems_lfs_ops;




int rtems_lfs_initialize(
  rtems_filesystem_mount_table_entry_t *mt_entry,
  void *data
)
{
  int status;
  size_t min_write_size = 0;
  rtems_lfs_mount_data *lfs_mount_data = data;
  rtems_flashdev_ioctl_page_info pg_info;

  if ( lfs_mount_data == NULL ) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }

  struct lfs_config * pCfg = (struct lfs_config *) lfs_mount_data->lfs_config;

  if (( pCfg == NULL ) || ( lfs_mount_data->flashdev == NULL )) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }

  rtems_lfs_fs_info_t   *fs_info = NULL;
  fs_info = (rtems_lfs_fs_info_t *) calloc(1, sizeof(rtems_lfs_fs_info_t));
  
  if ( NULL == fs_info ) {
    rtems_set_errno_and_return_minus_one( ENOMEM );
  }


  if ( lfs_mount_data->flashdev->get_min_write_block_size(
        lfs_mount_data->flashdev,
        &min_write_size) )
  {
    free(fs_info);
    rtems_set_errno_and_return_minus_one( EIO );
  }

  pg_info.location = 0;
  if (ioctl(lfs_mount_data->flashdev_fd, 
            RTEMS_FLASHDEV_IOCTL_GET_PAGEINFO_BY_OFFSET, &pg_info))
  {
    free(fs_info);
    rtems_set_errno_and_return_minus_one( EIO );
  }

  if( 0 == min_write_size)
  {
    min_write_size = pg_info.page_info.size;
  }

  if (!(lfs_mount_data->region.size % pg_info.erase_info.size) ||
      !(lfs_mount_data->region.offset % pg_info.erase_info.size) )
  {
    memcpy(&fs_info->ctx.lfs_config, pCfg,  sizeof(struct lfs_config));
    fs_info->ctx.lfs_config.read_size = (lfs_size_t) min_write_size;
    fs_info->ctx.lfs_config.prog_size = (lfs_size_t) min_write_size;
    fs_info->ctx.lfs_config.block_size = (lfs_size_t) pg_info.erase_info.size;
    fs_info->ctx.lfs_config.block_count = 
      (lfs_size_t) lfs_mount_data->region.size / pg_info.erase_info.size;
    fs_info->ctx.lfs_config.read = lfs_flashdev_read;
    fs_info->ctx.lfs_config.prog = lfs_flashdev_prog;
    fs_info->ctx.lfs_config.erase = lfs_flashdev_erase;
    fs_info->ctx.lfs_config.sync = lfs_flashdev_sync;
    fs_info->ctx.lfs_config.context = (void*) &fs_info->ctx;
    fs_info->ctx.flashdev_fd = lfs_mount_data->flashdev_fd;
    fs_info->ctx.flashdev = lfs_mount_data->flashdev;
    memcpy(&fs_info->ctx.region, &lfs_mount_data->region, 
            sizeof(rtems_flashdev_region));
    fs_info->ctx.lfs_config.context = (void*) &fs_info->ctx;

    status = lfs_mount(&fs_info->lfs, &fs_info->ctx.lfs_config);

    if (( status != 0) &&  lfs_mount_data->formating_allowed) {
      printf("LittleFS: Initial mount of LittleFS failed. Trying to format first");
      status = lfs_format(&fs_info->lfs, &fs_info->ctx.lfs_config);

      if (0 == status) {
        status = lfs_mount(&fs_info->lfs, &fs_info->ctx.lfs_config);
      }
      else
      {
        printf("LittleFS: Format failed");
      }

      if (status == 0)
      {
        rtems_recursive_mutex_init(&fs_info->s_mutex, "LFS_MTX");
        mt_entry->mt_fs_root->location.node_access = mt_entry->target;
        mt_entry->fs_info = fs_info;
        mt_entry->ops = &rtems_lfs_ops;
        mt_entry->mt_fs_root->location.handlers = &rtems_lfs_dir_handlers;
      } else {
        rtems_set_errno_and_return_minus_one( EIO );
        free(fs_info);
      }
    }
    return status;
  }

  free(fs_info);
  rtems_set_errno_and_return_minus_one( EIO );

}
