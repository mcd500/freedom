/* Copyright (c) 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-License-Identifier: GPL-2.0-or-later */
/* See the file LICENSE for further information */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "gpt.h"
#include "kprintf.h"

#define _ASSERT_SIZEOF(type, size) \
  _Static_assert(sizeof(type) == (size), #type " must be " #size " bytes wide")
#define _ASSERT_OFFSETOF(type, member, offset) \
  _Static_assert(offsetof(type, member) == (offset), #type "." #member " must be at offset " #offset)


typedef struct
{
  gpt_guid partition_type_guid;
  gpt_guid partition_guid;
  uint64_t first_lba;
  uint64_t last_lba;
  uint64_t attributes;
  uint16_t name[36];  // UTF-16
} gpt_partition_entry;
_ASSERT_SIZEOF(gpt_partition_entry, 128);


static inline bool guid_equal(const gpt_guid* a, const gpt_guid* b)
{
  for (int i = 0; i < GPT_GUID_SIZE; i++) {
    if (a->bytes[i] != b->bytes[i]) {
      return false;
    }
  }
  return true;
}


// Public functions

/**
 * Search the given block of partition entries for a partition with the given
 * GUID. Return a range of [0, 0] to indicate that the partition was not found.
 */
gpt_partition_range gpt_find_partition_by_guid(const void* entries, const gpt_guid* guid, uint32_t num_entries)
{
  gpt_partition_entry* gpt_entries = (gpt_partition_entry*) entries;
  for (uint32_t i = 0; i < num_entries; i++) {
    if (guid_equal(&gpt_entries[i].partition_type_guid, guid)) {
      kprintf("Found partition: 0x%x\r\n",i+1);
      return (gpt_partition_range) {
        .first_lba = gpt_entries[i].first_lba,
        .last_lba = gpt_entries[i].last_lba,
      };
    }
  }
  return (gpt_partition_range) { .first_lba = 0, .last_lba = 0 };
}
