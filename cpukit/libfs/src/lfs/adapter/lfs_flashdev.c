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

#include <dev/flash/flashdev.h>
#include <rtems/lfs.h>
#include <sys/ioctl.h>
#include "lfs_flashdev.h"
#include "lfs_adapter.h"

typedef enum 
{
	LS_ABORT = 1,
	LS_OBTAIN_FLASH,
	LS_GET_ACTIVE_PARTITION,
	LS_DEACTIVATE_PARTITION,
	LS_ACTIVATE_PARTITION,
	LS_RETURN
} lock_stages_t;

typedef enum 
{
	HS_ABORT = 1,
	HS_DEACTIVATE_PARTITION,
	HS_ACTIVATE_PARTITION,
	HS_RETURN
} unlock_stages_t;

static int32_t lock_flashdev_and_set_partition(
	rtems_flashdev * flashdev, 
	int32_t fd, 
	int32_t * partition_idx, 
	int32_t * partition_idx_store
)
{
	int32_t status = 0;
	bool done = false;
	lock_stages_t stage = LS_OBTAIN_FLASH;

	while ( !done ) {
		switch ( stage ) {
			case LS_OBTAIN_FLASH:
				status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_OBTAIN, NULL);
				break;
			case LS_GET_ACTIVE_PARTITION:
			  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_GET_ACTIVATE_PARTITION_IDX,
				  partition_idx_store);
				break;
			case LS_DEACTIVATE_PARTITION:
				if (*partition_idx_store >= 0){
					status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_DEACTIVATE_PARTITION,
					  partition_idx_store);
				}
				break;
			case LS_ACTIVATE_PARTITION:
				if (*partition_idx >= 0){
			  	status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_ACTIVATE_PARTITION, 
					  partition_idx);
				}
				break;
			case LS_RETURN:
				status = 0;
				done = true;
				break;
			default:
			case LS_ABORT:
				ioctl(fd, RTEMS_FLASHDEV_IOCTL_RELEASE, NULL);
				done = true;
				status = -1;
				break;
		}
		if ( (status < 0) && !done )
		{
			stage = LS_ABORT;
		} else {
			stage++;
		}
	}
	return status;
}

static int32_t unlock_flashdev_and_restore_partition(
	rtems_flashdev * flashdev,
	int32_t fd,
	int32_t * partition_idx,
	int32_t * partition_idx_store
)
{
	int32_t status = 0;
	bool done = false;
	lock_stages_t stage = LS_OBTAIN_FLASH;

	while ( !done ) {
		switch ( stage ) {
			case HS_DEACTIVATE_PARTITION:
				if (*partition_idx >= 0){
					status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_DEACTIVATE_PARTITION,
					  partition_idx);
				}
				break;
			case HS_ACTIVATE_PARTITION:
				if (*partition_idx_store >= 0){
			  	status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_ACTIVATE_PARTITION,
					  partition_idx_store);
				}
				break;
			case HS_RETURN:
				status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_RELEASE, NULL);
				done = true;
				break;
			default:
			case HS_ABORT:
				ioctl(fd, RTEMS_FLASHDEV_IOCTL_RELEASE, NULL);
				done = true;
				status = -1;
				break;
		}
		if ( (status < 0) && !done )
		{
			stage = HS_ABORT;
		} else {
			stage++;
		}
	}
	return status;
}

int lfs_flashdev_read(
  const struct lfs_config *c,
	lfs_block_t block,
	lfs_off_t off,
	void *buffer,
	lfs_size_t size
)
{
	int32_t store = 0;
	int32_t ret = 0;
	uint32_t address;

  if (c != NULL)
  {
    rtems_lfs_context_t* ctx = (rtems_lfs_context_t*) c->context;
    rtems_flashdev * flashdev = ctx->flashdev;
		
		if (ctx->flashdev_partition_idx >= 0)
		{
			if ( 0 == lock_flashdev_and_set_partition(flashdev, 
				ctx->flashdev_fd, &ctx->flashdev_partition_idx, &store ) )
			{
				address = (uint32_t) block * 
										(uint32_t) ctx->lfs_config.block_size + (uint32_t) off;

				ret = flashdev->read(flashdev, (uintptr_t) address, (size_t) size, 
								buffer);
				
				unlock_flashdev_and_restore_partition(flashdev, ctx->flashdev_fd, 
					&ctx->flashdev_partition_idx, &store );
				return ret;
			}
		}
		else
		{
			address = (uint32_t) ctx->partition.offset + (uint32_t) block * 
									(uint32_t) ctx->lfs_config.block_size + (uint32_t) off;

			return flashdev->read(flashdev, (uintptr_t) address, (size_t) size, 
												buffer);
				
		}
  }

  return LFS_ERR_INVAL;
}


int lfs_flashdev_prog(
	const struct lfs_config *c,
	lfs_block_t block,
	lfs_off_t off,
	const void *buffer,
	lfs_size_t size
)
{
	int32_t store = 0;
	int32_t ret = 0;
	uint32_t address;

  if (c != NULL)
  {
    rtems_lfs_context_t* ctx = (rtems_lfs_context_t*) c->context;
    rtems_flashdev * flashdev = ctx->flashdev;

		if (ctx->flashdev_partition_idx >= 0)
		{

			if ( 0 == lock_flashdev_and_set_partition(flashdev, 
				ctx->flashdev_fd, &ctx->flashdev_partition_idx, &store ) )
			{
				uint32_t address = (uint32_t) block * 
														(uint32_t) ctx->lfs_config.block_size + (uint32_t) off;
				
				ret = flashdev->write(flashdev, (uintptr_t) address, (size_t) size, 
								buffer);
				
				unlock_flashdev_and_restore_partition(flashdev, ctx->flashdev_fd, 
					&ctx->flashdev_partition_idx, &store );

				return ret;
			}
		} else {
			address = (uint32_t) ctx->partition.offset + (uint32_t) block * 
									(uint32_t) ctx->lfs_config.block_size + (uint32_t) off;

			return flashdev->write(flashdev, (uintptr_t) address, (size_t) size, 
								buffer);
		}
  }

  return LFS_ERR_INVAL;
}

int lfs_flashdev_erase(const struct lfs_config *c, lfs_block_t block)
{
	int32_t store = 0;
	int32_t ret = 0;
	uint32_t address;

  if (c != NULL)
  {
    rtems_lfs_context_t* ctx = (rtems_lfs_context_t*) c->context;
    rtems_flashdev * flashdev = ctx->flashdev;

		if (ctx->flashdev_partition_idx >= 0)
		{

			if ( 0 == lock_flashdev_and_set_partition(flashdev, 
				ctx->flashdev_fd, &ctx->flashdev_partition_idx, &store ) )
			{
				address = (uint32_t) block * (uint32_t) ctx->lfs_config.block_size;
				
				ret = flashdev->erase(flashdev, (uintptr_t) address, 
								(size_t) ctx->lfs_config.block_size);
				
				unlock_flashdev_and_restore_partition(flashdev, ctx->flashdev_fd, 
					&ctx->flashdev_partition_idx, &store );
				return ret;
			}
		} else {
			address = (uint32_t) ctx->partition.offset + (uint32_t) block * 
									(uint32_t) ctx->lfs_config.block_size;

			return flashdev->erase(flashdev, (uintptr_t) address, 
								(size_t) ctx->lfs_config.block_size);
		}
  }

  return LFS_ERR_INVAL;
}


int lfs_flashdev_sync(const struct lfs_config *c)
{
  return 0;
}
