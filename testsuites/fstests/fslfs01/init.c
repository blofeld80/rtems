/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * Copyright (c) 2013 embedded brains GmbH & Co. KG
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
#include <tmacros.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>

#include <rtems/lfs.h>
#include <rtems/libio.h>
//#include <rtems/libcsupport.h>

#include "../../libtests/flashdev01/test_flashdev.h"
#include "lfs_flashdev.h"

const char rtems_test_name[] = "LFS_SUPPORT 1";
const char BASE_FOR_TEST[] = "/mnt";

/* For LittleFS, the erase size translates to the LitteLF block size.
 * The block size must be a multiple of the cache size. The cache size
 * must in turn be a multiple of the read / write size
 */

#define PAGE_COUNT     64
#define PAGE_SIZE      512           /* lfs read and prog size */
#define ERASE_SIZE     4096          /* lfs block size */
#define LFS_CACHE_SIZE 1024          /* lfs cache size */

/* The lfs lookahead buffer can track 8 blocks per byte and 
 * must be a multiple of 8.
 */
#define LFS_LOOK_AHEAD_BUFFER_SIZE 8

#define TEST_DATA_SIZE (PAGE_SIZE * PAGE_COUNT)

uint8_t g_lfs_read_cache[LFS_CACHE_SIZE];
uint8_t g_lfs_prog_cache[LFS_CACHE_SIZE];
uint8_t g_lfs_look_ahead_buffer[LFS_LOOK_AHEAD_BUFFER_SIZE] __attribute__((aligned(32)));


char buff[TEST_DATA_SIZE] = {0};
const char flash_path[] = "/dev/flashdev0";
rtems_flashdev* flashdev;
FILE *file;
int fd;

void initialize_flashdev(void) 
{
  int status;
  rtems_flashdev_region region;

  flashdev = test_flashdev_init(PAGE_COUNT, PAGE_SIZE, ERASE_SIZE, PAGE_SIZE);

  /* Register the flashdev as a device */
  status = rtems_flashdev_register(flashdev, flash_path);
  rtems_test_assert(!status);

  /* Open the flashdev */
  file = fopen(flash_path, "r+");
  rtems_test_assert(file != NULL);

  /* Adjust the file buffering */
  status = setvbuf(file, NULL, _IOFBF, PAGE_SIZE);
  rtems_test_assert(!status);

  fd = fileno(file);

  /* Test Regions*/
  region.offset = 0;
  region.size = TEST_DATA_SIZE / 2;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_SET_REGION, &region);
  rtems_test_assert(!status);

  region.offset = TEST_DATA_SIZE / 2;
  region.size = TEST_DATA_SIZE / 2;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_SET_REGION, &region);
  rtems_test_assert(!status);

}


void deinitialize_flashdev(void) 
{
  int status;
  /* Close the file handle */
  status = fclose(file);
  rtems_test_assert(!status);

  /* Deregister path */
  status = rtems_flashdev_deregister(flash_path);
  rtems_test_assert(!status);

  test_flashdev_deinit(flashdev);
}


void mount_lfs(void)
{
  int status;
  rtems_lfs_mount_data mount_data;
  struct lfs_config lfs_config = {0};

  /*LittleFS Configuration*/
  lfs_config.context = NULL;
  lfs_config.block_cycles = 500;
  lfs_config.lookahead_size = LFS_LOOK_AHEAD_BUFFER_SIZE;
  lfs_config.cache_size  = LFS_CACHE_SIZE;
  lfs_config.read_buffer = (void*) g_lfs_read_cache;
  lfs_config.prog_buffer = (void*) g_lfs_prog_cache;
  lfs_config.lookahead_buffer = (void*) g_lfs_look_ahead_buffer;

  /*Set up mount data*/
  mount_data.flashdev = flashdev;
  mount_data.region.offset = TEST_DATA_SIZE / 2;
  mount_data.region.size = TEST_DATA_SIZE / 2;
  mount_data.formating_allowed = false;
  mount_data.lfs_config = (void*) &lfs_config;
  mount_data.flashdev_fd = fd;

  status = mount_and_make_target_path(
    NULL,
    BASE_FOR_TEST,
    RTEMS_FILESYSTEM_TYPE_LITTLEFS,
    RTEMS_FILESYSTEM_READ_WRITE,
    &mount_data
  );
  rtems_test_assert(status);

  mount_data.formating_allowed = true;
  status = mount_and_make_target_path(
    NULL,
    BASE_FOR_TEST,
    RTEMS_FILESYSTEM_TYPE_LITTLEFS,
    RTEMS_FILESYSTEM_READ_WRITE,
    &mount_data
  );
  rtems_test_assert(!status);
  
}

void mount_lfs(void)
{

}


static void run_test(void) 
{
  initialize_flashdev();
  mount_lfs();
  unmount_lfs();
  deinitialize_flashdev();
}

void Init(rtems_task_argument arg)
{
  TEST_BEGIN();

  run_test();

  TEST_END();
  rtems_test_exit(0);
}



#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_SIMPLE_CONSOLE_DRIVER

#define CONFIGURE_FILESYSTEM_LITTLEFS

#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 40

#define CONFIGURE_MAXIMUM_TASKS 2

#define CONFIGURE_INIT_TASK_STACK_SIZE (32 * 1024)
#define CONFIGURE_INIT_TASK_ATTRIBUTES RTEMS_FLOATING_POINT

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
