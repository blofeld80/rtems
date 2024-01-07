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



int lfs_flashdev_read(
const struct lfs_config *c,
lfs_block_t block,
lfs_off_t off,
void *buffer,
lfs_size_t size
)
{
  uint32_t address;

  if (c != NULL)
  {
    rtems_lfs_context_t* ctx = (rtems_lfs_context_t*) c->context;
    rtems_flashdev * flashdev = ctx->flashdev;

    address = (uint32_t) ctx->region.offset + (uint32_t) block * 
              (uint32_t) ctx->lfs_config.block_size + (uint32_t) off;

    return flashdev->read(flashdev, (uintptr_t) address, (size_t) size, 
                           buffer);
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
  uint32_t address;

  if (c != NULL)
  {
    rtems_lfs_context_t* ctx = (rtems_lfs_context_t*) c->context;
    rtems_flashdev * flashdev = ctx->flashdev;
    address = (uint32_t) ctx->region.offset + (uint32_t) block * 
              (uint32_t) ctx->lfs_config.block_size + (uint32_t) off;

    return flashdev->write(flashdev, (uintptr_t) address, (size_t) size, 
                           buffer);
  }

  return LFS_ERR_INVAL;
}

int lfs_flashdev_erase(const struct lfs_config *c, lfs_block_t block)
{
  uint32_t address;

  if (c != NULL)
  {
    rtems_lfs_context_t* ctx = (rtems_lfs_context_t*) c->context;
    rtems_flashdev * flashdev = ctx->flashdev;

    address = (uint32_t) ctx->region.offset + (uint32_t) block * 
              (uint32_t) ctx->lfs_config.block_size;

    return flashdev->erase(flashdev, (uintptr_t) address, 
                           (size_t) ctx->lfs_config.block_size);
  }
  return LFS_ERR_INVAL;
}


int lfs_flashdev_sync(const struct lfs_config *c)
{
  return 0;
}
