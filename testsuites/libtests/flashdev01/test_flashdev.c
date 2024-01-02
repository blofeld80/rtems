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

#include "test_flashdev.h"

#include <stdlib.h>
#include <string.h>

#include <rtems/seterr.h>

static size_t g_test_data_size = 0;
static size_t g_page_count = 0;
static size_t g_page_size = 0;
static size_t g_min_write_size = 0;
static size_t g_erase_size = 0;

/**
 * This flash device driver is for testing flashdev
 * API calls.
 */
typedef struct test_flashdev {
  char* data;
  uint32_t jedec_id;
} test_flashdev;

int test_flashdev_get_page_by_off(
  rtems_flashdev *flash,
  off_t search_offset,
  off_t *page_offset,
  size_t *page_size
);

int test_flashdev_get_page_by_index(
  rtems_flashdev *flash,
  off_t search_index,
  off_t *page_offset,
  size_t *page_size
);

int test_flashdev_get_page_count(
  rtems_flashdev *flash,
  int *page_count
);

int test_flashdev_get_min_write_size(
  rtems_flashdev *flash,
  size_t *min_write_size
);

int test_flashdev_get_erase_size(
  rtems_flashdev *flash,
  size_t *erase_size
);

uint32_t test_flashdev_get_jedec_id(
  rtems_flashdev* flash
);

int test_flashdev_get_type(
  rtems_flashdev* flash,
  rtems_flashdev_flash_type* type
);

int test_flashdev_read(
  rtems_flashdev* flash,
  uintptr_t offset,
  size_t count,
  void* buffer
);

int test_flashdev_write(
  rtems_flashdev* flash,
  uintptr_t offset,
  size_t count,
  const void* buffer
);

int test_flashdev_erase(
  rtems_flashdev* flash,
  uintptr_t offset,
  size_t count
);

/* Get page info by offset handler */
int test_flashdev_get_page_by_off(
  rtems_flashdev *flash,
  off_t search_offset,
  off_t *page_offset,
  size_t *page_size
)
{
  *page_offset = search_offset - (search_offset%g_page_size);
  *page_size = g_page_size;
  return 0;
}

/* Get page info by index handler */
int test_flashdev_get_page_by_index(
  rtems_flashdev *flash,
  off_t search_index,
  off_t *page_offset,
  size_t *page_size
)
{
  *page_offset = search_index * g_page_size;
  *page_size = g_page_size;
  return 0;
}

/* Get page count handler */
int test_flashdev_get_page_count(
  rtems_flashdev *flash,
  int *page_count
)
{
  *page_count = g_page_count;
  return 0;
}

/* Get min. write size handler */
int test_flashdev_get_min_write_size(
  rtems_flashdev *flash,
  size_t *min_write_size
)
{
  *min_write_size = g_min_write_size;
  return 0;
}

/* Get min. erase size handler */
int test_flashdev_get_erase_size(
  rtems_flashdev *flash,
  size_t *erase_size
)
{
  *erase_size = g_erase_size;
  return 0;
}


/* JEDEC ID handler, this would normally require a READID
 * call to the physical flash device.
 */
uint32_t test_flashdev_get_jedec_id(
  rtems_flashdev* flash
)
{
  test_flashdev* driver = flash->driver;
  return driver->jedec_id;
}

/* Function to identify what kind of flash is attached. */
int test_flashdev_get_type(
  rtems_flashdev *flash,
  rtems_flashdev_flash_type *type
)
{
  *type = RTEMS_FLASHDEV_NOR;
  return 0;
}

/* Read flash call. Any offset or count protections are
 * required to be done in the driver function. */
int test_flashdev_read(
  rtems_flashdev* flash,
  uintptr_t offset,
  size_t count,
  void* buffer
)
{
  test_flashdev* driver = flash->driver;

  if (offset + count > g_test_data_size) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }

  memcpy(buffer, &driver->data[offset], count);
  return 0;
}

/* Write Flash call. Any offset or count protections are
 * required to be done in the driver function. */
int test_flashdev_write(
  rtems_flashdev* flash,
  uintptr_t offset,
  size_t count,
  const void* buffer
)
{
  test_flashdev* driver = flash->driver;

  if (offset + count > g_test_data_size) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }

  memcpy(&driver->data[offset], buffer, count);
  return 0;
}

/* Erase Flash call. Any offset or count protections are
 * required to be done in the driver function. */
int test_flashdev_erase(
  rtems_flashdev* flash,
  uintptr_t offset,
  size_t count
)
{
  test_flashdev* driver = flash->driver;

  if (offset + count > g_test_data_size) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }

  if (offset%g_page_size || count%g_page_size) {
    rtems_set_errno_and_return_minus_one( EINVAL );
  }

  memset(&driver->data[offset], 0, count);
  return 0;
}

/* Initialize Flashdev and underlying driver. */
rtems_flashdev* test_flashdev_init(size_t page_size, size_t page_count, size_t min_write_size, size_t erase_size)
{
  if (0 == min_write_size) {
    return NULL;
  }

  if (0 == erase_size) {
    return NULL;
  }

  if (0 == page_size) {
    return NULL;
  }

  if (0 == page_count) {
    return NULL;
  }


  if (erase_size % min_write_size) {
    return NULL;
  }

  if (erase_size % page_size) {
    return NULL;
  }


  g_page_count = page_count;
  g_page_size = page_size;
  g_test_data_size = g_page_count * g_page_size;
  g_min_write_size = min_write_size;
  g_erase_size = erase_size;

  if (g_test_data_size % g_erase_size) {
    return NULL;
  }

  rtems_flashdev *flash = rtems_flashdev_alloc_and_init(sizeof(rtems_flashdev));

  if (flash == NULL) {
    return NULL;
  }

  test_flashdev* flash_driver = calloc(1, sizeof(test_flashdev));

  if (flash_driver == NULL) {
    rtems_flashdev_destroy_and_free(flash);
    return NULL;
  }

  flash_driver->data = calloc(1, g_test_data_size);
  if (flash_driver->data == NULL) {
    free(flash_driver);
    rtems_flashdev_destroy_and_free(flash);
    return NULL;
  }

  flash_driver->jedec_id = 0x00ABCDEF;

  flash->driver = flash_driver;
  flash->read = &test_flashdev_read;
  flash->write = &test_flashdev_write;
  flash->erase = &test_flashdev_erase;
  flash->get_jedec_id = &test_flashdev_get_jedec_id;
  flash->get_flash_type = &test_flashdev_get_type;
  flash->get_page_info_by_offset = &test_flashdev_get_page_by_off;
  flash->get_page_info_by_index = &test_flashdev_get_page_by_index;
  flash->get_page_count = &test_flashdev_get_page_count;
  flash->get_min_write_size = &test_flashdev_get_min_write_size;
  flash->get_erase_size = &test_flashdev_get_erase_size;

  return flash;
}

/* Free Flashdev and underlying driver. */
void test_flashdev_deinit(
  rtems_flashdev* flash
)
{
  if (NULL != flash)
  {
    if (NULL != flash->driver)
    {
      test_flashdev* flash_driver = (test_flashdev*) flash->driver;
      if (NULL != flash_driver->data)
      {
        free( flash_driver->data);
      }
      free(flash->driver);
    }
    rtems_flashdev_destroy_and_free(flash);
  }
}
