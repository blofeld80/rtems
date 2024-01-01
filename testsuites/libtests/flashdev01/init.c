/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
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

#include "tmacros.h"

#include "test_flashdev.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>



#define TEST_DATA_SIZE (PAGE_SIZE * PAGE_COUNT)
#define PAGE_COUNT 16
#define PAGE_SIZE 128
#define ERASE_SIZE 1024

const char rtems_test_name[] = "FLASHDEV 1";
const char test_string[] = "My test string!";

static void run_test(void);

static void run_test(void) {

  char buff[TEST_DATA_SIZE] = {0};
  FILE *file;
  int fd;
  rtems_flashdev* flash;
  int status;
  char* read_data;
  size_t bytes_read;
  rtems_flashdev_partition e_args;
  rtems_flashdev_ioctl_page_info pg_info;
  rtems_flashdev_partition partition;
  uint32_t jedec;
  int page_count;
  int type;
  const size_t min_write_size_in[] = {1,8,16};
  size_t min_write_size_out = 0;
  size_t erase_size = 0;
  const char flash_path[] = "/dev/flashdev0";
  const int page_count_in = 16;
  const int page_size_in = 128;


  for ( int loop = 0; loop <= 2; loop++)
  {
    /* Initalize the flash device driver and flashdev */
    flash = test_flashdev_init(page_size_in, page_count_in, min_write_size_in[loop], ERASE_SIZE);
    rtems_test_assert(flash != NULL);

    /* Register the flashdev as a device */
    status = rtems_flashdev_register(flash, flash_path);
    rtems_test_assert(!status);

    /* Open the flashdev */
    file = fopen(flash_path, "r+");
    rtems_test_assert(file != NULL);

    /* Adjust the file buffering */
    status = setvbuf(file, NULL, _IOFBF, min_write_size_in[loop]);
    rtems_test_assert(!status);

    fd = fileno(file);

    /* Read data from flash */
    read_data = fgets(buff, TEST_DATA_SIZE, file);
    rtems_test_assert(read_data != NULL);

    /* Fseek to start of flash and read again */
    status = fseek(file, 0x0, SEEK_SET);
    rtems_test_assert(!status);
    bytes_read = fread(buff, 1, TEST_DATA_SIZE, file);
    rtems_test_assert(bytes_read == TEST_DATA_SIZE);

    /* Fseek to start of flash */
    status = fseek(file, 0x0, SEEK_SET);
    rtems_test_assert(!status);

    /* Write the test name to the flash */
    status = fwrite(test_string, 1, sizeof(test_string), file);
    rtems_test_assert(status == sizeof(test_string));

    /* Fseek to start of flash and read again */
    status = fseek(file, 0x0, SEEK_SET);
    rtems_test_assert(!status);
    fgets(buff, TEST_DATA_SIZE, file);
    rtems_test_assert(!strncmp(buff, test_string, sizeof(test_string)));

    /* Test getting erase size */
    status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_GET_ERASE_SIZE, &erase_size);
    rtems_test_assert(!status);
    rtems_test_assert(ERASE_SIZE == erase_size);

    /* Test Erasing - this one must fail*/
    e_args.offset = 0x0;
    e_args.size = PAGE_SIZE;
    status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_ERASE, &e_args);
    rtems_test_assert(status);

    /* Test Erasing - this one must fail*/
    e_args.offset = 0x1;
    e_args.size = 
    ERASE_SIZE;
    status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_ERASE, &e_args);
    rtems_test_assert(status);

    /* Test Erasing*/
    e_args.offset = 0x0;
    e_args.size = ERASE_SIZE;
    status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_ERASE, &e_args);
    rtems_test_assert(!status);

    fseek(file, 0x0, SEEK_SET);
    fgets(buff, TEST_DATA_SIZE, file);
    rtems_test_assert(buff[0] == 0);

    /* Test getting JEDEC ID */
    status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_GET_JEDEC_ID, &jedec);
    rtems_test_assert(!status);
    rtems_test_assert(jedec == 0x00ABCDEF);

    /* Test getting flash type */
    status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_GET_TYPE, &type);
    rtems_test_assert(!status);
    rtems_test_assert(type == RTEMS_FLASHDEV_NOR);

    /* Test getting page info from offset */
    pg_info.location = PAGE_SIZE + PAGE_SIZE/2;

    status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_GET_PAGEINFO_BY_OFFSET, &pg_info);
    rtems_test_assert(!status);
    rtems_test_assert(pg_info.page_info.offset == PAGE_SIZE);
    rtems_test_assert(pg_info.page_info.size == PAGE_SIZE);

    /* Test getting page info from index */
    pg_info.location = 2;
    status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_GET_PAGEINFO_BY_INDEX, &pg_info);
    rtems_test_assert(!status);
    rtems_test_assert(pg_info.page_info.offset == 2*PAGE_SIZE);
    rtems_test_assert(pg_info.page_info.size == PAGE_SIZE);

    /* Test getting page count */
    status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_GET_PAGE_COUNT, &page_count);
    rtems_test_assert(!status);
    rtems_test_assert(page_count == PAGE_COUNT);

    /* Test getting min write size */
    status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_GET_MIN_WRITE_SIZE, &min_write_size_out);
    rtems_test_assert(!status);
    rtems_test_assert(0 == memcmp(&min_write_size_out, &min_write_size_in[loop] , sizeof(size_t)));

    /* Close the file handle */
    status = fclose(file);
    rtems_test_assert(!status);

    /* Deregister path */
    status = rtems_flashdev_deregister(flash_path);
    rtems_test_assert(!status);

    test_flashdev_deinit(flash);
  }

  /* Initalize the flash device driver and flashdev */
  flash = test_flashdev_init(page_size_in, page_count_in, min_write_size_in[1], ERASE_SIZE);
  rtems_test_assert(flash != NULL);

  /* Register the flashdev as a device */
  status = rtems_flashdev_register(flash, flash_path);
  rtems_test_assert(!status);

  /* Open the flashdev */
  file = fopen(flash_path, "r+");
  rtems_test_assert(file != NULL);

  /* Adjust the file buffering */
  status = setvbuf(file, NULL, _IOFBF, min_write_size_in[1]);
  rtems_test_assert(!status);

  fd = fileno(file);

  /* Prepare the flash */
  memset(buff,0x55,TEST_DATA_SIZE);
  status = fwrite(buff, 1, TEST_DATA_SIZE, file);
  rtems_test_assert(status == TEST_DATA_SIZE);
  memset(buff,0x00,TEST_DATA_SIZE);

  /* Fseek to start of flash and read again */
  status = fseek(file, 0x0, SEEK_SET);
  rtems_test_assert(!status);
  bytes_read = fread(buff, 1, TEST_DATA_SIZE, file);
  rtems_test_assert(bytes_read == TEST_DATA_SIZE);
  memset(buff,0x00,TEST_DATA_SIZE);

  /* Test Partitions - this one must fail */
  partition.offset = ERASE_SIZE;
  partition.size = 0x200;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_CREATE_PARTITION, &partition);
  rtems_test_assert(status);

  /* Test Partitions - this one must fail*/
  partition.offset = 0x200;
  partition.size = ERASE_SIZE;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_CREATE_PARTITION, &partition);
  rtems_test_assert(status);

  /* Test Partitions - create partition 0*/
  partition.offset = 0;
  partition.size = ERASE_SIZE;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_CREATE_PARTITION, &partition);
  rtems_test_assert(status == 0);

  /* Activate Partition 0*/
  status = 0;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_ACTIVATE_PARTITION, &status);
  rtems_test_assert(!status);

  /* Try to resize partition 0 above limit*/
  partition.offset = 1;
  partition.size = ERASE_SIZE;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_RESIZE_PARTITION, &partition);
  rtems_test_assert(status);

  /* Try to resize partition 0 within limit*/
  partition.offset = 0;
  partition.size = 2*ERASE_SIZE;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_RESIZE_PARTITION, &partition);
  rtems_test_assert(status == 0);

  /* Try to resize partition 0 to original size*/
  partition.size = ERASE_SIZE;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_RESIZE_PARTITION, &partition);
  rtems_test_assert(status == 0);

  /* Deactivate Partition 0*/
  status = 0;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_DEACTIVATE_PARTITION, &status);
  rtems_test_assert(!status);

  /* Test Partitions - ioctl returns partition idx*/
  partition.offset = ERASE_SIZE;
  partition.size = ERASE_SIZE;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_CREATE_PARTITION, &partition);
  rtems_test_assert(status == 1);

  /* Activate Partition - wrong partion idx*/
  status = 3;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_ACTIVATE_PARTITION, &status);
  rtems_test_assert(status);

  /* Activate Partition - ioctl returns partition idx*/
  status = 1;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_ACTIVATE_PARTITION, &status);
  rtems_test_assert(!status);

  /* Test Erasing - ioctl returns partition idx*/
  e_args.offset = 0x0;
  e_args.size = ERASE_SIZE;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_ERASE, &e_args);
  rtems_test_assert(!status);

  /* Test read within then partition */
  status = fseek(file, 0x0, SEEK_SET);
  rtems_test_assert(!status);
  bytes_read = fread(buff, 1, 0x200, file);
  rtems_test_assert(bytes_read == 0x200);

  /* Test read to larger then partition */
  fseek(file, 0x0, SEEK_SET);
  rtems_test_assert(!status);
  read_data = fgets(buff, 2048, file);
  rtems_test_assert(buff[0] == 0);

  /* Test fseek outside of partition */
  fseek(file, 0x0, SEEK_SET);
  rtems_test_assert(!status);
  status = fseek(file, ERASE_SIZE+1, SEEK_SET);
  rtems_test_assert(status);

  /* Write to base unset partition and check the writes location */
  fseek(file, 0x0, SEEK_SET);
  fwrite("HELLO WORLD!!!!!", 1, 16, file);

  /* delete partition 0 */
  status = 0;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_DELETE_PARTITION, &status);
  rtems_test_assert(!status);

  /* delete partition 1 */
  status = 1;
  status = ioctl(fd, RTEMS_FLASHDEV_IOCTL_DELETE_PARTITION, &status);
  rtems_test_assert(!status);


  /* Test read within then partition */
  status = fseek(file, 0x0, SEEK_SET);
  rtems_test_assert(!status);
  bytes_read = fread(buff, 1, 0x200, file);
  rtems_test_assert(bytes_read == 0x200);
  rtems_test_assert(&buff[0] == memchr(buff, 0x55, 0x200));

  fseek(file, ERASE_SIZE, SEEK_SET);
  fgets(buff, 16, file);
  rtems_test_assert(strncmp(buff, "HELLO WORLD!!!!!", 16));

  /* Close the file handle */
  status = fclose(file);
  rtems_test_assert(!status);

  /* Deregister path */
  status = rtems_flashdev_deregister(flash_path);
  rtems_test_assert(!status);

  test_flashdev_deinit(flash);
}

static void Init(rtems_task_argument arg)
{
  TEST_BEGIN();

  run_test();

  TEST_END();
  rtems_test_exit(0);
}

#define CONFIGURE_MICROSECONDS_PER_TICK 2000

#define CONFIGURE_APPLICATION_DOES_NOT_NEED_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 7

#define CONFIGURE_MAXIMUM_TASKS 2

#define CONFIGURE_MAXIMUM_SEMAPHORES 1

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT

#include <rtems/confdefs.h>
