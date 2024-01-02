/*
 * Copyright (C) 2024 Bernd Moessner
 * Copyright (C) 2023 Aaron Nyholm
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <dev/flash/flashdev.h>

#include <rtems/imfs.h>
#include <rtems/score/assert.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>

#define RTEMS_FLASHDEV_MAX_PARTITIONS 16
#define RTEMS_FLASHDEV_PARTITION_ALLOC_FULL 0xFFFFFFFFUL
#define RTEMS_FLASHDEV_PARTITION_UNDEFINED 0xFFFFFFFFUL


static inline uint32_t set_bit(uint32_t in, uint32_t bit_idx)
{
  return in | ( 1 << bit_idx );
}

static inline uint32_t clear_bit(uint32_t in, uint32_t bit_idx)
{
  return in & ~( 1 << (bit_idx) );
}


/* IOCTL Functions*/
static uint32_t rtems_flashdev_ioctl_get_jedec_id(
  rtems_flashdev *flash
);

static int rtems_flashdev_ioctl_erase(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
);

static int rtems_flashdev_ioctl_create_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
);

static int rtems_flashdev_ioctl_delete_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
);

static int rtems_flashdev_ioctl_resize_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
);

static int rtems_flashdev_ioctl_activate_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
);

static int rtems_flashdev_ioctl_deactivate_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
);

static int rtems_flashdev_ioctl_get_active_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
);

static uint32_t rtems_flashdev_ioctl_get_flash_type(
  rtems_flashdev *flash,
  void *arg
);

static int rtems_flashdev_ioctl_get_pageinfo_by_offset(
  rtems_flashdev *flash,
  void *arg
);

static int rtems_flashdev_ioctl_get_pageinfo_by_index(
  rtems_flashdev *flash,
  void *arg
);

static int rtems_flashdev_ioctl_get_page_count(
  rtems_flashdev *flash,
  void *arg
);

static int rtems_flashdev_ioctl_get_min_write_size(
  rtems_flashdev *flash,
  void *arg
);

static int rtems_flashdev_ioctl_get_erase_size(
  rtems_flashdev *flash,
  void *arg
);


static int rtems_flashdev_do_init(
  rtems_flashdev *flash,
  void ( *destroy )( rtems_flashdev *flash )
);

static int rtems_flashdev_read_write(
  rtems_libio_t *iop,
  const void *write_buff,
  void *read_buff,
  size_t count
);

static ssize_t rtems_flashdev_read(
  rtems_libio_t *iop,
  void *buffer,
  size_t count
);

static ssize_t rtems_flashdev_write(
  rtems_libio_t *iop,
  const void *buffer,
  size_t count
);

static off_t rtems_flashdev_get_partition_offset(
  rtems_flashdev *flash,
  rtems_libio_t *iop
);

static size_t rtems_flashdev_get_partition_size(
  rtems_flashdev *flash,
  rtems_libio_t *iop
);

static int rtems_flashdev_create_partition(
  rtems_libio_t *iop,
  rtems_flashdev_partition *partition_table,
  uint32_t partition_idx,
  rtems_flashdev_partition *region_in
);


static int rtems_flashdev_get_addr(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  size_t count,
  off_t *addr
);

static int rtems_flashdev_get_abs_addr(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  size_t count,
  off_t *addr
);

static int rtems_flashdev_update_and_return(
  rtems_libio_t *iop,
  int status,
  size_t count
);

static int rtems_flashdev_check_partition_valid(
  rtems_flashdev *flash,
  rtems_flashdev_partition * region
);

static uint32_t rtems_flashdev_find_unallocated_partition(
  rtems_libio_t *iop
);

static int rtems_flashdev_open(
  rtems_libio_t *iop,
  const char *path,
  int oflag,
  mode_t mode
);

static int rtems_flashdev_close(
  rtems_libio_t *iop
);

static int rtems_flashdev_ioctl(
  rtems_libio_t *iop,
  ioctl_command_t command,
  void *arg
);

static off_t rtems_flashdev_lseek(
  rtems_libio_t *iop,
  off_t offset,
  int whence
);

static void rtems_flashdev_node_destroy(
  IMFS_jnode_t *node
);

static uint32_t rtems_flashdev_get_defined_partitions(
  rtems_libio_t *iop
);

static bool rtems_flashdev_is_a_partition_active(
  rtems_libio_t *iop
);

static bool rtems_flashdev_is_partition_defined(
  rtems_libio_t *iop,
  uint32_t partition_idx
);

static int rtems_flashdev_activate_partition(
  rtems_libio_t *iop,
  uint32_t partition_idx
);

static int rtems_flashdev_deactivate_partition(
  rtems_libio_t *iop,
  uint32_t partition_idx
);

static void rtems_flashdev_mark_partition_defined(
  rtems_libio_t *iop,
  uint32_t partition_idx
);

static void rtems_flashdev_mark_partition_undefined(
  rtems_libio_t *iop,
  uint32_t partition_idx
);

static bool rtems_flashdev_check_partition_overlap(
  rtems_libio_t *iop,
  rtems_flashdev_partition *partition_table,
  rtems_flashdev_partition *region_in
);

static int rtems_flashdev_check_partition_offset(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  off_t offset
);

static uint32_t rtems_flashdev_get_active_partition_index(
  rtems_libio_t *iop
);

static void rtems_flashdev_obtain( rtems_flashdev *flash );

static void rtems_flashdev_release( rtems_flashdev *flash );

static const rtems_filesystem_file_handlers_r rtems_flashdev_handler = {
  .open_h = rtems_flashdev_open,
  .close_h = rtems_flashdev_close,
  .read_h = rtems_flashdev_read,
  .write_h = rtems_flashdev_write,
  .ioctl_h = rtems_flashdev_ioctl,
  .lseek_h = rtems_flashdev_lseek,
  .fstat_h = IMFS_stat,
  .ftruncate_h = rtems_filesystem_default_ftruncate,
  .fsync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fcntl_h = rtems_filesystem_default_fcntl,
  .kqfilter_h = rtems_filesystem_default_kqfilter,
  .mmap_h = rtems_filesystem_default_mmap,
  .poll_h = rtems_filesystem_default_poll,
  .readv_h = rtems_filesystem_default_readv,
  .writev_h = rtems_filesystem_default_writev };

static const IMFS_node_control
  rtems_flashdev_node_control = IMFS_GENERIC_INITIALIZER(
    &rtems_flashdev_handler,
    IMFS_node_initialize_generic,
    rtems_flashdev_node_destroy
);

static int rtems_flashdev_do_init(
  rtems_flashdev *flash,
  void ( *destroy )( rtems_flashdev *flash )
)
{
  char mtx_name[19];
  sprintf(mtx_name, "FDEV_MTX_%08x", (unsigned int) flash);
  rtems_recursive_mutex_init( &flash->mutex, (const char*) &mtx_name);
  flash->destroy = destroy;
  flash->read = NULL;
  flash->write = NULL;
  flash->erase = NULL;
  flash->get_jedec_id = NULL;
  flash->get_flash_type = NULL;
  flash->get_page_info_by_offset = NULL;
  flash->get_page_info_by_index = NULL;
  flash->get_page_count = NULL;
  flash->get_min_write_size = NULL;
  flash->get_erase_size = NULL;
  flash->partition_table = NULL;
  return 0;
}

static int rtems_flashdev_read_write(
  rtems_libio_t *iop,
  const void *write_buff,
  void *read_buff,
  size_t count
)
{
  rtems_flashdev *flash = IMFS_generic_get_context_by_iop( iop );
  off_t addr;
  int status;

  if ( read_buff == NULL && write_buff == NULL ) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }

  /* Get flash address */
  status = rtems_flashdev_get_addr( flash, iop, count, &addr );
  if ( status < 0 ) {
    return status;
  }

  /* Read or Write to flash */
  rtems_flashdev_obtain( flash );
  if ( read_buff != NULL ) {
    status = ( *flash->read )( flash, addr, count, read_buff );
  } else if ( write_buff != NULL ) {
    size_t min_write_size = 0;
    status = (flash)->get_min_write_size(flash, &min_write_size);

    if ( status < 0 ) {
      return status;
    }

    if (0 == min_write_size )
    {
      rtems_set_errno_and_return_minus_one( EIO );
    }
    else
    {
      if (count % min_write_size)
      {
        rtems_set_errno_and_return_minus_one( EINVAL );
      }
      if (addr % min_write_size)
      {
        rtems_set_errno_and_return_minus_one( EFAULT );
      }
    }

    status = ( *flash->write )( flash, addr, count, write_buff );
  }
  rtems_flashdev_release( flash );

  /* Update offset and return */
  return rtems_flashdev_update_and_return( iop, status, count );
}

static ssize_t rtems_flashdev_read(
  rtems_libio_t *iop,
  void *buffer,
  size_t count
)
{
  return rtems_flashdev_read_write( iop, NULL, buffer, count );
}

static ssize_t rtems_flashdev_write(
  rtems_libio_t *iop,
  const void *buffer,
  size_t count
)
{
  return rtems_flashdev_read_write( iop, buffer, NULL, count);
}

static off_t rtems_flashdev_get_partition_offset(
  rtems_flashdev *flash,
  rtems_libio_t *iop
)
{
  /* Region is already checked to be defined */
  assert( rtems_flashdev_get_active_partition_index( iop ) != RTEMS_FLASHDEV_PARTITION_UNDEFINED );
  rtems_flashdev_partition *table = flash->partition_table;
  return table[ rtems_flashdev_get_active_partition_index( iop ) ].offset;
}

static size_t rtems_flashdev_get_partition_size(
  rtems_flashdev *flash,
  rtems_libio_t *iop
)
{
  /* Region is already checked to be defined */
  assert( rtems_flashdev_get_active_partition_index( iop ) != RTEMS_FLASHDEV_PARTITION_UNDEFINED );
  rtems_flashdev_partition *table = flash->partition_table;
  return table[ rtems_flashdev_get_active_partition_index( iop ) ].size;
}

static int rtems_flashdev_get_addr(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  size_t count,
  off_t *addr
)
{
  off_t new_offset;

  /* Check address is in valid partition */
  new_offset = iop->offset + count;

  if (rtems_flashdev_check_partition_offset(flash, iop, new_offset)) {
    return -1;
  }

  /* Get address for operation */
  if ( !rtems_flashdev_is_a_partition_active( iop ) ) {
    *addr = iop->offset;
  } else {
    *addr = ( iop->offset + rtems_flashdev_get_partition_offset( flash, iop ) );
  }
  return 0;
}

static int rtems_flashdev_get_abs_addr(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  size_t count,
  off_t *addr
)
{
  off_t new_offset;

  /* Check address is in valid partition */
  new_offset = *addr + count;

  if (rtems_flashdev_check_partition_offset(flash, iop, new_offset)) {
    return -1;
  }

  /* Get address for operation */
  if ( rtems_flashdev_is_a_partition_active( iop ) ) {
    *addr = ( *addr + rtems_flashdev_get_partition_offset( flash, iop ) );
  }
  return 0;
}

static int rtems_flashdev_update_and_return(
  rtems_libio_t *iop,
  int status,
  size_t count
)
{
  /* Update offset and return */
  if ( status == 0 ) {
    iop->offset += count;
    return count;
  } else {
    rtems_set_errno_and_return_minus_one( status );
  }
}

static int rtems_flashdev_create_partition(
  rtems_libio_t *iop,
  rtems_flashdev_partition *partition_table,
  uint32_t partition_idx,
  rtems_flashdev_partition *partition_in
)
{
  int i = 0x0000FFFF & partition_idx;

  /* Set partitions values */
  partition_table[ i ].offset = partition_in->offset;
  partition_table[ i ].size = partition_in->size;

  rtems_flashdev_mark_partition_defined( iop, i );

  return i;
}

static int rtems_flashdev_check_partition_valid(
  rtems_flashdev *flash,
  rtems_flashdev_partition * region
)
{
  size_t erase_size = 0;
  int status = (flash)->get_erase_size(flash, &erase_size);

  if (0 != status)
  {
    return status;
  }
  if (region->offset % erase_size || region->size % erase_size)
  {
    return -1;
  }

  return 0;
}

static uint32_t rtems_flashdev_find_unallocated_partition(
  rtems_libio_t *iop
)
{
  uint16_t defined_partitions = rtems_flashdev_get_defined_partitions(iop);

  for (uint32_t idx = 0; idx < RTEMS_FLASHDEV_MAX_PARTITIONS; ++idx)
  {
    if (!(defined_partitions & ( 1 << idx )))
    {
      return idx;
    }
  }

  return RTEMS_FLASHDEV_PARTITION_ALLOC_FULL;
}

static int rtems_flashdev_open(
  rtems_libio_t *iop,
  const char *path,
  int oflag,
  mode_t mode
)
{
  int ret = rtems_filesystem_default_open( iop, path, oflag, mode );
  rtems_flashdev_mark_partition_defined(iop, RTEMS_FLASHDEV_PARTITION_UNDEFINED);
  return ret;
}

static int rtems_flashdev_close(
  rtems_libio_t *iop
)
{
  return rtems_filesystem_default_close( iop );
}

static int rtems_flashdev_ioctl(
  rtems_libio_t *iop,
  ioctl_command_t command,
  void *arg
)
{
  rtems_flashdev *flash = IMFS_generic_get_context_by_iop( iop );
  int err = 0;

  rtems_flashdev_obtain( flash );

  switch ( command ) {
    case RTEMS_FLASHDEV_IOCTL_OBTAIN:
      rtems_flashdev_obtain( flash );
      err = 0;
      break;
    case RTEMS_FLASHDEV_IOCTL_RELEASE:
      rtems_flashdev_release( flash );
      err = 0;
      break;
    case RTEMS_FLASHDEV_IOCTL_GET_JEDEC_ID:
      *( (uint32_t *) arg ) = rtems_flashdev_ioctl_get_jedec_id( flash );
      err = 0;
      break;
    case RTEMS_FLASHDEV_IOCTL_ERASE:
      err = rtems_flashdev_ioctl_erase( flash, iop, arg );
      break;
    case RTEMS_FLASHDEV_IOCTL_CREATE_PARTITION:
      err = rtems_flashdev_ioctl_create_partition( flash, iop, arg );
      break;
    case RTEMS_FLASHDEV_IOCTL_DELETE_PARTITION:
      err = rtems_flashdev_ioctl_delete_partition( flash, iop, arg );
      break;
    case RTEMS_FLASHDEV_IOCTL_RESIZE_PARTITION:
      err = rtems_flashdev_ioctl_resize_partition( flash, iop, arg );
    case RTEMS_FLASHDEV_IOCTL_ACTIVATE_PARTITION:
      err = rtems_flashdev_ioctl_activate_partition( flash, iop, arg );
      break;
    case RTEMS_FLASHDEV_IOCTL_DEACTIVATE_PARTITION:
      err = rtems_flashdev_ioctl_deactivate_partition( flash, iop, arg );
      break;
    case RTEMS_FLASHDEV_IOCTL_GET_ACTIVATE_PARTITION_IDX:
      err = rtems_flashdev_ioctl_get_active_partition( flash, iop, arg );
      break;
    case RTEMS_FLASHDEV_IOCTL_GET_TYPE:
      err = rtems_flashdev_ioctl_get_flash_type( flash, arg );
      break;
    case RTEMS_FLASHDEV_IOCTL_GET_PAGEINFO_BY_OFFSET:
      err = rtems_flashdev_ioctl_get_pageinfo_by_offset( flash, arg );
      break;
    case RTEMS_FLASHDEV_IOCTL_GET_PAGEINFO_BY_INDEX:
      err = rtems_flashdev_ioctl_get_pageinfo_by_index( flash, arg );
      break;
    case RTEMS_FLASHDEV_IOCTL_GET_PAGE_COUNT:
      err = rtems_flashdev_ioctl_get_page_count( flash, arg );
      break;
    case RTEMS_FLASHDEV_IOCTL_GET_MIN_WRITE_SIZE:
      err = rtems_flashdev_ioctl_get_min_write_size( flash, arg );
      break;
    case RTEMS_FLASHDEV_IOCTL_GET_ERASE_SIZE:
      err = rtems_flashdev_ioctl_get_erase_size( flash, arg );
      break;
    default:
      err = -EINVAL;
  }

  rtems_flashdev_release( flash );
  if ( err < 0 ) {
    rtems_set_errno_and_return_minus_one( err );
  }

  return err;
}

static off_t rtems_flashdev_lseek(
  rtems_libio_t *iop,
  off_t offset,
  int whence
)
{
  off_t tmp_offset;
  rtems_flashdev *flash = IMFS_generic_get_context_by_iop( iop );

  if ( offset < 0 ) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }

  switch ( whence ) {
    case SEEK_CUR:
      tmp_offset = iop->offset + offset;
      break;
    case SEEK_SET:
      tmp_offset = offset;
      break;
    case SEEK_END:
    default:
      rtems_set_errno_and_return_minus_one( EINVAL );
  }

  if ( ( rtems_flashdev_is_a_partition_active(iop) ) &&
       ( tmp_offset > rtems_flashdev_get_partition_size( flash, iop ) ) ) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }

  iop->offset = tmp_offset;
  return iop->offset;
}

static void rtems_flashdev_node_destroy(
  IMFS_jnode_t *node
)
{
  rtems_flashdev *flash;

  flash = IMFS_generic_get_context_by_node( node );

  ( *flash->destroy )( flash );

  IMFS_node_destroy_default( node );
}

static uint32_t rtems_flashdev_get_defined_partitions(
  rtems_libio_t *iop
)
{
  return 0x0000FFFF & ((uint32_t) iop->data0);
}

static bool rtems_flashdev_is_a_partition_active(
  rtems_libio_t *iop
)
{
  return (rtems_flashdev_get_active_partition_index( iop )
    != RTEMS_FLASHDEV_PARTITION_UNDEFINED);
}

static bool rtems_flashdev_is_partition_defined(
  rtems_libio_t *iop,
  uint32_t partition_idx
)
{
  uint32_t defined_partitions = rtems_flashdev_get_defined_partitions(iop);
  return (defined_partitions & ( 1 << partition_idx ));
}

static int rtems_flashdev_activate_partition(
  rtems_libio_t *iop,
  uint32_t partition_idx
)
{
  if(!rtems_flashdev_is_partition_defined(iop, partition_idx)){return -1;}
  iop->data0 = set_bit(iop->data0, partition_idx + RTEMS_FLASHDEV_MAX_PARTITIONS);
  return 0;
}

static int rtems_flashdev_deactivate_partition(
  rtems_libio_t *iop,
  uint32_t partition_idx
)
{
  if(!rtems_flashdev_is_partition_defined(iop, partition_idx)){return -1;}
  iop->data0 = clear_bit(iop->data0, partition_idx + RTEMS_FLASHDEV_MAX_PARTITIONS);
  return 0;
}

static void rtems_flashdev_mark_partition_defined(
  rtems_libio_t *iop,
  uint32_t partition_idx
)
{
  iop->data0 = set_bit(iop->data0, partition_idx);
}

static void rtems_flashdev_mark_partition_undefined(
  rtems_libio_t *iop,
  uint32_t partition_idx
)
{
  if (rtems_flashdev_is_a_partition_active( iop ))
  {
    if( partition_idx == rtems_flashdev_get_active_partition_index( iop ) )
    {
      rtems_flashdev_deactivate_partition(iop, partition_idx);
    }
  }
  iop->data0 = clear_bit(iop->data0, partition_idx);
}

static bool rtems_flashdev_check_partition_overlap(
  rtems_libio_t *iop,
  rtems_flashdev_partition *partition_table,
  rtems_flashdev_partition *partition_in
)
{
  off_t cp_start;
  off_t cp_end;
  off_t partition_in_start = partition_in->offset;
  off_t partition_in_end = partition_in->offset + partition_in->size;
  uint16_t defined_partitions = rtems_flashdev_get_defined_partitions(iop);

  for (uint16_t idx = 0; idx < RTEMS_FLASHDEV_MAX_PARTITIONS; ++idx)
  {
    if (defined_partitions & ( 1 << idx ))
    {
      cp_start = partition_table[ idx ].offset;
      cp_end = partition_table[ idx ].offset
        + partition_table[ idx ].size;

      if( (partition_in_start < cp_end) && (partition_in_end > cp_start) )
      {
        return true;
      }
    }
  }
  return false;
}

static int rtems_flashdev_check_partition_offset(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  off_t offset
)
{
  if ( ( rtems_flashdev_is_a_partition_active( iop ) ) &&
       ( offset > rtems_flashdev_get_partition_size( flash, iop ) ) ) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }
  return 0;
}

static uint32_t rtems_flashdev_get_active_partition_index(
  rtems_libio_t *iop
)
{
  uint32_t active = ((uint32_t)iop->data0) >> RTEMS_FLASHDEV_MAX_PARTITIONS;

  for (uint32_t idx = 0; idx < RTEMS_FLASHDEV_MAX_PARTITIONS; idx++)
  {
    if (active & ( 1 << idx ))
    {
      return 0x0000FFFF & idx;
    }
  }

  return RTEMS_FLASHDEV_PARTITION_UNDEFINED;
}

static void rtems_flashdev_obtain( rtems_flashdev *flash )
{
  rtems_recursive_mutex_lock( &flash->mutex );
}

static void rtems_flashdev_release( rtems_flashdev *flash )
{
  rtems_recursive_mutex_unlock( &flash->mutex );
}

/* API Implementation*/
rtems_flashdev *rtems_flashdev_alloc_and_init( size_t size )
{
  rtems_flashdev *flash = NULL;

  if ( size >= sizeof( *flash ) ) {
    flash = calloc( 1, size );
    if ( NULL != flash ) {
      int rv;
      rtems_flashdev_partition * table = calloc( RTEMS_FLASHDEV_MAX_PARTITIONS, sizeof(rtems_flashdev_partition));
      rv = rtems_flashdev_do_init( flash, rtems_flashdev_destroy_and_free );
      if ( (rv != 0) || (table == NULL) ) {
        rtems_recursive_mutex_destroy( &flash->mutex );
        free( flash );
        if  (NULL != table)
        {
          free( table );
        }
        return NULL;
      }
      flash->partition_table = table;
    }
  }

  return flash;
}

int rtems_flashdev_init( rtems_flashdev *flash )
{
  memset( flash, 0, sizeof( *flash ) );
  return rtems_flashdev_do_init( flash, rtems_flashdev_destroy );
}

int rtems_flashdev_register(
  rtems_flashdev *flash,
  const char *flash_path
)
{
  int rv;

  rv = IMFS_make_generic_node(
    flash_path,
    S_IFCHR | S_IRWXU | S_IRWXG | S_IRWXO,
    &rtems_flashdev_node_control,
    flash
  );

  if ( rv != 0 ) {
    ( *flash->destroy )( flash );
  }

  return rv;
}

int rtems_flashdev_deregister(
  const char *flash_path
)
{
  rtems_filesystem_eval_path_context_t ctx;
  int eval_flags = RTEMS_FS_FOLLOW_LINK;
  const rtems_filesystem_location_info_t *currentloc =
    rtems_filesystem_eval_path_start( &ctx , flash_path, eval_flags );

  return IMFS_rmnod(NULL, currentloc);
}

void rtems_flashdev_destroy( rtems_flashdev *flash )
{
  rtems_recursive_mutex_destroy( &flash->mutex );
}

void rtems_flashdev_destroy_and_free( rtems_flashdev *flash )
{
  if ( NULL == flash ) {
    return;
  }

  if (NULL != flash->partition_table )
  {
    free( flash->partition_table );
  }

  rtems_recursive_mutex_destroy( &( flash->mutex ) );
  free( flash );
  return;
}

static uint32_t rtems_flashdev_ioctl_get_jedec_id( rtems_flashdev *flash )
{
  if ( flash->get_jedec_id == NULL ) {
    return 0;
  } else {
    return ( *flash->get_jedec_id )( flash );
  }
}

static int rtems_flashdev_ioctl_erase(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
)
{
  rtems_flashdev_partition *erase_args_1;
  off_t new_offset;
  int status;

  if ( flash->erase == NULL ) {
    return 0;
  }

  erase_args_1 = (rtems_flashdev_partition *) arg;
  /* Check erasing valid partition */
  if ( 0 != rtems_flashdev_check_partition_valid(flash, erase_args_1))
  {
    return EINVAL;
  }

  new_offset = erase_args_1->offset;
  status = rtems_flashdev_get_abs_addr(flash, iop, erase_args_1->size, &new_offset);
  if ( status < 0 ) {
    return status;
  }

  /* Erase flash */
  status = ( *flash->erase )( flash, new_offset, erase_args_1->size );
  return status;
}


static int rtems_flashdev_ioctl_create_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
)
{
  rtems_flashdev_partition *partition_in;
  rtems_flashdev_partition *table = flash->partition_table;
  partition_in = (rtems_flashdev_partition *) arg;
  uint32_t partition_idx = 0;

  if (table == NULL) {
    rtems_set_errno_and_return_minus_one( ENOMEM );
  }

  if ( 0 != rtems_flashdev_check_partition_valid(flash, partition_in))
  {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }

  if (rtems_flashdev_check_partition_overlap(iop, table, partition_in)) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }
  partition_idx = rtems_flashdev_find_unallocated_partition(iop);

  if (RTEMS_FLASHDEV_PARTITION_ALLOC_FULL == partition_idx)
  {
    /* New partition to allocate and all partitions allocated */
    rtems_set_errno_and_return_minus_one( ENOMEM );
  }

  /* New partition to allocate and space to allocate partition */
  return rtems_flashdev_create_partition( iop, table, partition_idx, partition_in );
}

static int rtems_flashdev_ioctl_delete_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
)
{
  uint32_t *partition_idx = (uint32_t*) arg;

  if (flash->partition_table == NULL) {
    rtems_set_errno_and_return_minus_one( ENOMEM );
  }

  /* Check partition to clear */
  if ( *partition_idx >=  RTEMS_FLASHDEV_MAX_PARTITIONS ) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }

  /* Clear partition */
  rtems_flashdev_mark_partition_undefined( iop, *partition_idx );
  return 0;
}

static int rtems_flashdev_ioctl_resize_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
)
{
  rtems_flashdev_partition storage;

  uint32_t partition_idx = 0;
  int ret = -1;
  if ( !rtems_flashdev_is_a_partition_active( iop ) ) {
    return -1;
  }

  partition_idx = rtems_flashdev_get_active_partition_index( iop );
  storage = flash->partition_table[ partition_idx ];

  if ( 0 == rtems_flashdev_ioctl_delete_partition( flash, iop, &partition_idx ))
  {
    ret = rtems_flashdev_ioctl_create_partition( flash, iop, arg);

    if (ret >= 0)
    {
      rtems_flashdev_activate_partition(iop, (uint32_t) ret);
      return  ret;
    }
    else
    {
      ret = rtems_flashdev_create_partition( iop, flash->partition_table, partition_idx, &storage);
      if (ret >= 0)
      {
        rtems_flashdev_activate_partition(iop, partition_idx);
        return  -1;
      }
    }
  }

  return -1;
}

static int rtems_flashdev_ioctl_activate_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
)
{
  uint32_t *partition_idx = (uint32_t*) arg;
  return rtems_flashdev_activate_partition(iop, *partition_idx);
}

static int rtems_flashdev_ioctl_deactivate_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
)
{
  uint32_t *partition_idx = (uint32_t*) arg;
  return rtems_flashdev_deactivate_partition(iop, *partition_idx);
}

static int rtems_flashdev_ioctl_get_active_partition(
  rtems_flashdev *flash,
  rtems_libio_t *iop,
  void *arg
)
{
  int32_t * partition_idx = (int32_t *) arg;
  *partition_idx = -1;
  if (rtems_flashdev_is_a_partition_active( iop ))
  {
    *partition_idx = rtems_flashdev_get_active_partition_index( iop );
  }

}

static uint32_t rtems_flashdev_ioctl_get_flash_type(
  rtems_flashdev *flash,
  void *arg
)
{
  rtems_flashdev_flash_type *type = (rtems_flashdev_flash_type*)arg;
  if ( flash->get_flash_type == NULL ) {
    return 0;
  } else {
    return ( *flash->get_flash_type )( flash, type );
  }
}

static int rtems_flashdev_ioctl_get_pageinfo_by_offset(
  rtems_flashdev *flash,
  void *arg
)
{
  rtems_flashdev_ioctl_page_info *page_info;

  if ( arg == NULL ) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }
  if ( flash->get_page_info_by_offset == NULL ) {
    return 0;
  } else {
    page_info = (rtems_flashdev_ioctl_page_info *) arg;
    return ( *flash->get_page_info_by_offset )( flash,
                                         page_info->location,
                                         &page_info->page_info.offset,
                                         &page_info->page_info.size );
  }
}

static int rtems_flashdev_ioctl_get_pageinfo_by_index( rtems_flashdev *flash,
                                                void *arg )
{
  rtems_flashdev_ioctl_page_info *page_info;

  if ( arg == NULL ) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }
  if ( flash->get_page_info_by_index == NULL ) {
    return 0;
  } else {
    page_info = (rtems_flashdev_ioctl_page_info *) arg;
    return ( *flash->get_page_info_by_index )( flash,
                                           page_info->location,
                                           &page_info->page_info.offset,
                                           &page_info->page_info.size );
  }
}

static int rtems_flashdev_ioctl_get_page_count( rtems_flashdev *flash, void *arg )
{
  if ( arg == NULL ) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }
  if ( flash->get_page_count == NULL ) {
    return 0;
  } else {
    return ( *flash->get_page_count )( flash, ( (int *) arg ) );
  }
}

static int rtems_flashdev_ioctl_get_min_write_size(
  rtems_flashdev *flash,
  void *arg
)
{
  if ( arg == NULL ) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }
  if ( flash->get_min_write_size == NULL ) {
    return 0;
  } else {
    return ( *flash->get_min_write_size )( flash, ( (size_t *) arg ) );
  }
}

static int rtems_flashdev_ioctl_get_erase_size(
  rtems_flashdev *flash,
  void *arg
)
{
  if ( arg == NULL ) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }
  if ( flash->get_erase_size == NULL ) {
    return 0;
  } else {
    return ( *flash->get_erase_size )( flash, ( (size_t *) arg ) );
  }
}