/* Copyright (c) 2020, Daniel Kasza
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "ext2_types.h"

#include <stdbool.h>
#include <stdlib.h>

/* The maximum number of cache blocks.
 * You can override this by defining this macro when compiling the code, but this value must match between the client
 * and the library.
 *
 * Increasing this also increases the size of the ext2_fs_t structure, so this is not free.
 *
 * In practice, using a single cache line is really awful, but there is also no need for megabytes of cache.
 * The default number (4) was selected based on the benchmark of reading an 8MB file from a filesystem using 4KB blocks.
 * This is expected to be the common use case for this library.
 * For future reference, the cache utilization was:
 *    1 cache block:  1034 hits, 2002 misses
 *    2 cache blocks: 3022 hits, 14 misses
 *    4 cache blocks: 3022 hits, 14 misses
 *    8 cache blocks: 3023 hits, 13 misses
 *   16 cache blocks: 3024 hits, 12 misses
 *  128 cache blocks: 3025 hits, 11 misses
 * As you can see, adding more cache blocks beyond 2 does not significantly increase the hit ratio.
 * The default value of 4 was picked because 2 cache blocks would not be enough for large files due to triply-indirect
 * blocks, and also to reduce the chance of thrashing because two blocks map to the same cache lines.
 */
#ifndef EXT2_FS_CACHE_BLOCKS_COUNT_MAX
#define EXT2_FS_CACHE_BLOCKS_COUNT_MAX 4
#endif

/* Ext2 APIs.
 *
 * Error handling:
 *   Functions that can return errors return "const char *".
 *   On success, the value returned will be NULL.
 *   On failure, the value returned will be a string describing why the call failed.
 *   The assumption is that in the systems where this library will be used, there will be no need to handle specific
 *   failures in different ways, but the user still needs to know why a call failed.
 */

/* Disk access callback type. */
typedef const char *(*ext2_disk_access_func_t)(
    /* User defined context pointer. */
    void *context,
    /* ID of the first 512B sector to read. */
    uint32_t first,
    /* Number of 512B sectors to read. */
    uint32_t count,
    /* The sectors will be copied here. */
    uint8_t *buffer
);

/* State of the library.
 * Clients should not directly access or modify the fields of this structure.
 */
typedef struct {
    ext2_disk_access_func_t disk_access;
    void *context;

    uint32_t blocks_count;
    uint32_t block_size;
    uint32_t log2_block_size;
    uint32_t block_offset_mask;
    uint32_t blocks_per_group;
    uint32_t inodes_per_group;

    void     *cache_blocks;
    size_t    cache_blocks_count;
    uint32_t  cache_tags[EXT2_FS_CACHE_BLOCKS_COUNT_MAX];

    /* Cache performance counters.
     * These are only updated on accesses that are eligible for caching.
     */
    uint32_t cache_hits;
    uint32_t cache_misses;
} ext2_fs_t;

/* Open a filesystem. */
const char *ext2_open_fs(
    /* Disk access callback and context pointer. */
    ext2_disk_access_func_t disk_access,
    void *context,

    /* Chunk of memory to use as disk cache.
     * The cache has to be large enough for at least one block of the filesystem.
     */
    void *cache_memory,
    size_t cache_memory_size,

    /* Pointer to state structure.
     * The structure must be allocated by the user, so this must not be NULL.
     */
    ext2_fs_t *fs
);

/* Get an inode by index.
 *
 * This is roughly equivalent to opening a file.
 *
 * This function lets you access inodes that do not have hard links.
 * This is primarily for internal use, but it could be useful for others.
 */
const char *ext2_get_inode_by_idx(
    ext2_fs_t *fs,
    uint32_t inode_idx,
    ext2_inode_t *inode
);

/* Get an inode by path.
 *
 * This is roughly equivalent to opening a file.
 *
 * To avoid string manipulation, and string copying, the path is represented as a NULL terminated array of strings.
 * For example, to open "/somedir/file", you would pass { "somedir", "file", NULL }.
 */
const char *ext2_get_inode_by_path(
    ext2_fs_t *fs,
    const char **path_array,
    ext2_inode_t *inode
);

/* Read from a file.
 * This function lets you read the file regardless of its type.
 */
const char *ext2_read(
    /* The file is represented by the fs:inode pair.
     * You have to get the inode before you can read from a file.
     */
    ext2_fs_t *fs,
    ext2_inode_t *inode,

    /* Byte offset, and number of bytes to read. */
    uint32_t offset,
    uint32_t count,
    /* Buffer to read into. */
    uint8_t *buffer
);
