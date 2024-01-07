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

#include <rtems.h>
#include <rtems/libio.h>
#include <rtems/libio_.h>
#include <fcntl.h>
#include "lfs_adapter.h"



static void rtems_lfs_ops_lock(
    const rtems_filesystem_mount_table_entry_t * mt_entry
);
static void rtems_lfs_ops_unlock(
    const rtems_filesystem_mount_table_entry_t * mt_entry
);

/**
 * File operations
 */

int rtems_lfs_file_open(rtems_libio_t *iop, const char *path, int oflag,
  mode_t mode)
{
  int ret = 0;

  rtems_lfs_ops_lock(iop->pathinfo.mt_entry);

  rtems_lfs_fs_info_t *fs_info = iop->pathinfo.mt_entry->fs_info;
  lfs_t   *p_lfs  =   &fs_info->lfs;

  int32_t lfs_oflag = 0;

  int32_t off = (int32_t) strlen(iop->pathinfo.mt_entry->target);
  if (oflag == 0)
  {
    lfs_oflag |= LFS_O_RDONLY;
  }
  else
  {
    if (oflag & O_WRONLY)
    {
      lfs_oflag |= LFS_O_WRONLY;
    }
    if (oflag & O_RDWR)
    {
      lfs_oflag |= LFS_O_RDWR;
    }
    if (oflag & O_CREAT)
    {
      lfs_oflag |= LFS_O_CREAT;
    }
    if (oflag & O_APPEND)
    {
      lfs_oflag |= LFS_O_APPEND;
    }
    if (oflag & O_EXCL)
    {
      lfs_oflag |= LFS_O_EXCL;
    }
    if (oflag & O_TRUNC)
    {
      lfs_oflag |= LFS_O_TRUNC;
    }
  }

  ret = lfs_file_open(p_lfs, &fs_info->lfs_file, &path[off], lfs_oflag);

  iop->pathinfo.node_access = path;

  rtems_lfs_ops_unlock(iop->pathinfo.mt_entry);

  return ret;
}


int rtems_lfs_file_close(rtems_libio_t *iop)
{
  int ret = 0;
  rtems_lfs_ops_lock(iop->pathinfo.mt_entry);
  rtems_lfs_fs_info_t *fs_info = iop->pathinfo.mt_entry->fs_info;
  lfs_t   *p_lfs  =   &fs_info->lfs;
  ret =  lfs_file_close(p_lfs, &fs_info->lfs_file);
  rtems_lfs_ops_unlock(iop->pathinfo.mt_entry);
  return ret;
}


static ssize_t rtems_lfs_file_read(rtems_libio_t *iop, void *buf, size_t len)
{
  int ret = 0;
  rtems_lfs_ops_lock(iop->pathinfo.mt_entry);
  rtems_lfs_fs_info_t *fs_info = iop->pathinfo.mt_entry->fs_info;
  lfs_t   *p_lfs  =   &fs_info->lfs;
  ret =  lfs_file_read(p_lfs, &fs_info->lfs_file, buf, (lfs_size_t) len);
  if (ret == 0){ret = -1;}
  rtems_lfs_ops_unlock(iop->pathinfo.mt_entry);
  return ret;
}


static ssize_t rtems_lfs_file_write(rtems_libio_t *iop, const void *buf, size_t len)
{
  int ret = 0;
  rtems_lfs_ops_lock(iop->pathinfo.mt_entry);

  rtems_lfs_fs_info_t *fs_info = iop->pathinfo.mt_entry->fs_info;
  lfs_t   *p_lfs  =   &fs_info->lfs;

  ret =  lfs_file_write(p_lfs, &fs_info->lfs_file, buf, (lfs_size_t) len);
  rtems_lfs_ops_unlock(iop->pathinfo.mt_entry);
  return ret;
}

static int rtems_lfs_fstat(
  const rtems_filesystem_location_info_t *loc,
  struct stat *buf
)
{

  rtems_lfs_ops_lock(loc->mt_entry);
  rtems_lfs_fs_info_t   *fs_info = loc->mt_entry->fs_info;

  //buf->st_dev = fs_info->fat.vol.dev;
  //buf->st_ino = fat_fd->ino;
    buf->st_rdev = 0ll;

  //regular file, 777,everyone, all others
  buf->st_mode  = S_IFREG | S_IRWXU | S_IRWXG | S_IRWXO;
  buf->st_size = (fs_info->ctx.lfs_config.block_size * fs_info->ctx.lfs_config.block_count);
  buf->st_blocks = buf->st_size >> 9; //devide by 512
  buf->st_blksize = fs_info->ctx.lfs_config.block_size;

  memset(&buf->st_atime, 0, sizeof(struct timespec ));
  memset(&buf->st_ctime, 0, sizeof(struct timespec ));
  memset(&buf->st_mtime, 0, sizeof(struct timespec ));

  rtems_lfs_ops_unlock(loc->mt_entry);
  return 0;
}


static int rtems_lfs_file_ftruncate(rtems_libio_t *iop, off_t length)
{
  return 0;
}


static int rtems_lfs_file_sync(
  rtems_libio_t *iop
)
{
  printf("LittleFS: rtems_lfs_file_sync\n");
  errno = EINVAL;
  return -1;
}



// Directory operations

static ssize_t rtems_lfs_dir_read(rtems_libio_t *iop, void *buffer, size_t count)
{
  printf("LittleFS: rtems_lfs_dir_read\n");
  errno = ENOMEM;
  return -1;
}

static int rtems_lfs_dir_fstat(
  const rtems_filesystem_location_info_t *loc,
  struct stat *buf
)
{
  printf("LittleFS: rtems_lfs_dir_fstat\n");

  rtems_lfs_ops_lock(loc->mt_entry);

  rtems_lfs_fs_info_t *fs_info = loc->mt_entry->fs_info;
  lfs_t   *p_lfs  =   &fs_info->lfs;

  //buf->st_dev = fs_info->fat.vol.dev;
  //buf->st_ino = fat_fd->ino;
    buf->st_rdev = 0ll;

  //regular file, 777,everyone, all others
  buf->st_mode  = S_IFREG | S_IRWXU | S_IRWXG | S_IRWXO;
  buf->st_size = (fs_info->ctx.lfs_config.block_size * fs_info->ctx.lfs_config.block_count);
  buf->st_blocks = buf->st_size >> 9; //devide by 512
  buf->st_blksize = fs_info->ctx.lfs_config.block_size;

  memset(&buf->st_atime, 0, sizeof(struct timespec ));
  memset(&buf->st_ctime, 0, sizeof(struct timespec ));
  memset(&buf->st_mtime, 0, sizeof(struct timespec ));

  rtems_lfs_ops_unlock(loc->mt_entry);
  return 0;
}

/**
 * File operations table
 */
const rtems_filesystem_file_handlers_r rtems_lfs_file_handlers = {
  .open_h = rtems_lfs_file_open,
  .close_h = rtems_lfs_file_close,
  .read_h = rtems_lfs_file_read,
  .write_h = rtems_lfs_file_write,
  .ioctl_h = rtems_filesystem_default_ioctl,
  .lseek_h = rtems_filesystem_default_lseek_file,
  .fstat_h = rtems_lfs_fstat,
  .ftruncate_h = rtems_lfs_file_ftruncate,
  .fsync_h = rtems_lfs_file_sync,
  .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync, //Do we need this?
  .fcntl_h = rtems_filesystem_default_fcntl,
  .kqfilter_h = rtems_filesystem_default_kqfilter,
  .mmap_h = rtems_filesystem_default_mmap,
  .poll_h = rtems_filesystem_default_poll,
  .readv_h = rtems_filesystem_default_readv,
  .writev_h = rtems_filesystem_default_writev
};

/**
 * Directory operations table
 */
const rtems_filesystem_file_handlers_r rtems_lfs_dir_handlers = {
  .open_h = rtems_filesystem_default_open,
  .close_h = rtems_filesystem_default_close,
  .read_h = rtems_lfs_dir_read,
  .write_h = rtems_filesystem_default_write,
  .ioctl_h = rtems_filesystem_default_ioctl,
  .lseek_h = rtems_filesystem_default_lseek_directory,
  .fstat_h = rtems_lfs_dir_fstat,
  .ftruncate_h = rtems_filesystem_default_ftruncate_directory,
  .fsync_h = rtems_filesystem_default_fsync_or_fdatasync,                                            //Do we need this?
  .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,                                        //Do we need this?
  .fcntl_h = rtems_filesystem_default_fcntl,
  .kqfilter_h = rtems_filesystem_default_kqfilter,
  .mmap_h = rtems_filesystem_default_mmap,
  .poll_h = rtems_filesystem_default_poll,
  .readv_h = rtems_filesystem_default_readv,
  .writev_h = rtems_filesystem_default_writev
};




/**
 * Filesystem operations
 */
static void rtems_lfs_ops_lock(
    const rtems_filesystem_mount_table_entry_t * mt_entry
)
{
  rtems_lfs_fs_info_t *fs_info = mt_entry->fs_info;
  rtems_recursive_mutex_lock(&fs_info->s_mutex);
}


static void rtems_lfs_ops_unlock(
    const rtems_filesystem_mount_table_entry_t * mt_entry
)
{
  rtems_lfs_fs_info_t *fs_info = mt_entry->fs_info;
  rtems_recursive_mutex_unlock(&fs_info->s_mutex);
}


int rtems_lfs_ops_mount (rtems_filesystem_mount_table_entry_t * mt_entry)
{
  rtems_set_errno_and_return_minus_one( ENOTSUP );
}


int rtems_lfs_ops_unmount(rtems_filesystem_mount_table_entry_t * mt_entry)
{
  rtems_set_errno_and_return_minus_one( ENOTSUP );
}


static bool rtems_lfs_ops_are_nodes_equal(
  const rtems_filesystem_location_info_t *a,
  const rtems_filesystem_location_info_t *b
)
{
  return (a->node_access_2 == b->node_access_2);
}


static bool rtems_lfs_ops_eval_is_directory(
  rtems_filesystem_eval_path_context_t *ctx,
  void *arg
)
{
  printf("LittleFS: rtems_lfs_ops_eval_is_directory\n");
  rtems_filesystem_location_info_t *currentloc =
    rtems_filesystem_eval_path_get_currentloc(ctx);

  rtems_lfs_fs_info_t *fs_info = currentloc->mt_entry->fs_info;
  lfs_t   *p_lfs  =   &fs_info->lfs;

  struct lfs_info info;

  int ret = lfs_stat(p_lfs, ctx->path, &info);

  return (ret == 0) && (info.type == LFS_TYPE_DIR);
}


static rtems_filesystem_eval_path_generic_status rtems_lfs_ops_eval_token(
  rtems_filesystem_eval_path_context_t *ctx,
  void *arg,
  const char *token,
  size_t tokenlen
)
{
  printf("LittleFS: rtems_lfs_ops_eval_token\n");
  rtems_filesystem_eval_path_generic_status status =
    RTEMS_FILESYSTEM_EVAL_PATH_GENERIC_DONE;
  if (rtems_filesystem_is_current_directory(token, tokenlen)) {
    rtems_filesystem_eval_path_clear_token(ctx);
    status = RTEMS_FILESYSTEM_EVAL_PATH_GENERIC_CONTINUE;
  } else {
    rtems_filesystem_location_info_t *currentloc =
        rtems_filesystem_eval_path_get_currentloc(ctx);

    rtems_lfs_fs_info_t *fs_info = currentloc->mt_entry->fs_info;
    lfs_t   *p_lfs  =   &fs_info->lfs;

    struct lfs_info info;
    int ret = lfs_stat(p_lfs, token, &info);

    if (0 == ret) {
      rtems_filesystem_eval_path_clear_token(ctx);
      if(info.type == LFS_TYPE_DIR)
      {
        currentloc->handlers = &rtems_lfs_dir_handlers;
      }
      else
      {
        currentloc->node_access_2 = (void*) calloc(tokenlen+1, 1);
        memcpy(currentloc->node_access_2, token, tokenlen + 1);
        currentloc->handlers = &rtems_lfs_file_handlers;
      }

      if (rtems_filesystem_eval_path_has_path(ctx)) {
      status = RTEMS_FILESYSTEM_EVAL_PATH_GENERIC_CONTINUE;
      }
    } else if (LFS_ERR_NOENT == ret) {
      status = RTEMS_FILESYSTEM_EVAL_PATH_GENERIC_NO_ENTRY;
    } else {
      rtems_filesystem_eval_path_error(ctx, lfs_error_to_errno(ret));
    }
  }

  return status;
}


static const rtems_filesystem_eval_path_generic_config lfs_eval_config = {
  .is_directory = rtems_lfs_ops_eval_is_directory,
  .eval_token = rtems_lfs_ops_eval_token
};


static void rtems_lfs_ops_eval_path(rtems_filesystem_eval_path_context_t *ctx)
{
  int test = 0;
  rtems_filesystem_eval_path_generic(ctx, NULL, &lfs_eval_config);
}


static int rtems_lfs_ops_link(
  const rtems_filesystem_location_info_t *parentloc,
  const rtems_filesystem_location_info_t *targetloc,
  const char *name,
  size_t namelen
)
{
  printf("LittleFS: rtems_lfs_ops_link\n");
  return 0;
}


static int rtems_lfs_ops_mk_node(
    const rtems_filesystem_location_info_t *parentloc,
    const char *name,
    size_t namelen,
    mode_t mode,
    dev_t dev
)
{

  int ret = 0;
  rtems_lfs_ops_lock(parentloc->mt_entry);

  rtems_lfs_fs_info_t *fs_info = parentloc->mt_entry->fs_info;
  lfs_t   *p_lfs  =   &fs_info->lfs;

    if (S_ISDIR(mode))
    {
      ret = lfs_mkdir(p_lfs, name);
    }
    else if (S_ISREG(mode))
    {
      lfs_file_t  f;
      ret = lfs_file_open(p_lfs, &f, name, LFS_O_CREAT);
      if(0 == ret)
      {
        ret = lfs_file_close(p_lfs, &f);
      }
    }

    printf("LittleFS: rtems_lfs_ops_mk_node\n");
    rtems_lfs_ops_unlock(parentloc->mt_entry);
    return ret;
}


static int rtems_lfs_ops_rm_node(const rtems_filesystem_location_info_t
    *parent_pathloc, const rtems_filesystem_location_info_t *pathloc)
{
    printf("LittleFS: rtems_lfs_ops_rm_node\n");
    rtems_lfs_ops_lock(pathloc->mt_entry);
  rtems_lfs_fs_info_t *fs_info = pathloc->mt_entry->fs_info;
  lfs_t   *p_lfs  =   &fs_info->lfs;

    int ret = lfs_remove(p_lfs, pathloc->node_access_2);
    rtems_lfs_ops_unlock(pathloc->mt_entry);
    return ret;
}


static int rtems_lfs_ops_clone_node(rtems_filesystem_location_info_t *loc)
{
  printf("LittleFS: rtems_lfs_ops_clone_node\n");
    return 0;
}


void rtems_lfs_ops_free_node(const rtems_filesystem_location_info_t *pathloc)
{
  printf("LittleFS: rtems_lfs_ops_free_node\n");
  rtems_lfs_ops_lock(pathloc->mt_entry);
  if(pathloc->node_access_2 != NULL)
  {
  free(pathloc->node_access_2);
  }
  rtems_lfs_ops_unlock(pathloc->mt_entry);
}


void rtems_lfs_ops_shut_down(rtems_filesystem_mount_table_entry_t *temp_mt_entry)
{
  return;
}


static int rtems_lfs_ops_utimens(
  const rtems_filesystem_location_info_t *loc,
  struct timespec times[2]
)
{
  printf("LittleFS: rtems_lfs_ops_utimens\n");
  return -1;
}


static int rtems_lfs_ops_rename(
  const rtems_filesystem_location_info_t *oldparentloc,
  const rtems_filesystem_location_info_t *oldloc,
  const rtems_filesystem_location_info_t *newparentloc,
  const char *name,
  size_t namelen
)
{
  printf("LittleFS: rtems_lfs_ops_rename\n");
  return -1;
}


static int rtems_lfs_ops_statvfs(
  const rtems_filesystem_location_info_t *__restrict loc,
  struct statvfs *__restrict buf
)
{
  printf("LittleFS: rtems_lfs_ops_statvfs\n");
  return 0;
}

/**
 * Filesystem operations table
 */
const rtems_filesystem_operations_table rtems_lfs_ops = {
  .lock_h = rtems_lfs_ops_lock,
  .unlock_h = rtems_lfs_ops_unlock,
  .eval_path_h = rtems_lfs_ops_eval_path,
  .link_h = rtems_lfs_ops_link,
  .are_nodes_equal_h = rtems_lfs_ops_are_nodes_equal,
  .mknod_h = rtems_lfs_ops_mk_node,
  .rmnod_h = rtems_lfs_ops_rm_node,
  .fchmod_h = rtems_filesystem_default_fchmod,
  .chown_h = rtems_filesystem_default_chown,
  .clonenod_h =  rtems_lfs_ops_clone_node,
  .freenod_h = rtems_lfs_ops_free_node,
  .mount_h = rtems_lfs_ops_mount,
  .unmount_h = rtems_lfs_ops_unmount,
  .fsunmount_me_h = rtems_lfs_ops_shut_down,
  .utimens_h = rtems_lfs_ops_utimens,
  .symlink_h = rtems_filesystem_default_symlink,
  .readlink_h = rtems_filesystem_default_readlink,
  .rename_h = rtems_lfs_ops_rename,
  .statvfs_h = rtems_lfs_ops_statvfs
};


