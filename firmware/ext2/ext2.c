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

#include "ext2.h"

#include <string.h>

#define CACHE_TAG_INVALID                                                                   ((uint32_t)(-1))

/* Cached access functions */
static const char *cached_read(
    ext2_fs_t *fs,
    uint64_t offset,
    uint64_t count,
    uint8_t *buffer
);

const char *ext2_open_fs(
    ext2_disk_access_func_t disk_access,
    void *context,
    void *cache_memory,
    size_t cache_memory_size,
    ext2_fs_t *fs
) {
    const char *error = NULL;

    /* Read the superblock.
     * To avoid wasting memory somewhere else, we use the cache memory to temporarily store the superblock.
     * The minimum block size of Ext2 is 1024B, which happens to be the size of the superblock, so this is safe.
     */
    if (cache_memory_size < 1024) {
        return "block cache is too small";
    }
    /* The superblock is always at 1024B offset. */
    error = disk_access(context, 2, 2, cache_memory);
    if (error) {
        return error;
    }

    /* At this point, we have the superblock.
     * Make sure we can handle this filesystem.
     */
    ext2_superblock_t *superblock = cache_memory;
    if (superblock->magic != EXT2_SUPERBLOCK_MAGIC) {
        return "not ext2";
    }
    if (superblock->rev_level != EXT2_SUPERBLOCK_REV_LEVEL_EXPECTED) {
        return "only ext2 rev0 is supported";
    }
    if ((1024 << superblock->log_block_size) > cache_memory_size) {
        return "block cache is too small";
    }

    /* Initialize the state structure. */
    fs->disk_access = disk_access;
    fs->context = context;

    fs->blocks_count      = superblock->blocks_count;
    fs->block_size        = (1024 << superblock->log_block_size);
    fs->log2_block_size   = (  10 +  superblock->log_block_size);
    fs->block_offset_mask = (1 << fs->log2_block_size) - 1;
    fs->blocks_per_group  = superblock->blocks_per_group;
    fs->inodes_per_group  = superblock->inodes_per_group;

    fs->cache_blocks = cache_memory;
    fs->cache_blocks_count = cache_memory_size >> fs->log2_block_size;
    if (fs->cache_blocks_count > EXT2_FS_CACHE_BLOCKS_COUNT_MAX) {
        fs->cache_blocks_count = EXT2_FS_CACHE_BLOCKS_COUNT_MAX;
    }

    uint32_t i;
    for (i=0; i<EXT2_FS_CACHE_BLOCKS_COUNT_MAX; i++) {
        fs->cache_tags[i] = CACHE_TAG_INVALID;
    }

    fs->cache_hits   = 0;
    fs->cache_misses = 0;

    return NULL;
}

static const char *get_block_group_descriptor(
    ext2_fs_t *fs,
    uint32_t group_idx,
    ext2_block_group_descriptor_t *descriptor
) {
    uint32_t number_of_groups = (fs->blocks_count + fs->blocks_per_group - 1) / fs->blocks_per_group;

    if (group_idx >= number_of_groups) {
        return "group does not exist";
    }

    /* The group descriptor table starts on the first block after the superblock. */
    uint64_t offset = fs->block_size;
    if (offset < 2048) {
        offset = 2048;
    }
    offset += (((uint64_t)sizeof(*descriptor)) * group_idx);

    const char *error = cached_read(
        fs,
        offset,
        sizeof(*descriptor),
        (uint8_t*)descriptor
    );

    return error;
}

const char *ext2_get_inode_by_idx(
    ext2_fs_t *fs,
    uint32_t inode_idx,
    ext2_inode_t *inode
) {
    const char *error = NULL;
    
    uint32_t group_idx = ext2_get_inode_group(inode_idx, fs->inodes_per_group);
    uint32_t local_idx = ext2_get_inode_local_idx(inode_idx, fs->inodes_per_group);

    /* Get the group descriptor, so we can find where the inode table starts. */
    ext2_block_group_descriptor_t group = { 0 };
    error = get_block_group_descriptor(fs, group_idx, &group);
    if (error) {
        return error;
    }

    /* Get the inode from the inode table. */
    uint64_t offset = (((uint64_t)group.inode_table) << fs->log2_block_size) + (sizeof(*inode) * local_idx);

    error = cached_read(
        fs,
        offset,
        sizeof(*inode),
        (uint8_t*)inode
    );

    return error;
}

/* Search a directory inode for a named inode. */
const char *search_directory(
    ext2_fs_t *fs,
    /* The new inode is also returned here! */
    ext2_inode_t *inode,
    const char *name
) {
    const char *error = NULL;

    if ((inode->mode & EXT2_INODE_MODE_IFMASK) != EXT2_INODE_MODE_IFDIR) {
        return "not a directory";
    }

    size_t namelen = strlen(name);
    if (namelen > 256) {
        return "name is too long";
    }

    /* Iterate over all directory entries, and check the name if the name length matches.
     * Note: this would be very slow without a cache.
     */
    uint32_t offset = 0;
    for (;;) {
        ext2_dirent_t entry = { 0 };
        
        error = ext2_read(fs, inode, offset, sizeof(entry), (uint8_t*)&entry);

        if (error) {
            return error;
        }

        if (entry.namelen == namelen) {
            /* This is a potential match. We have to load the full name and compare. */
            char entry_name[namelen];

            error = ext2_read(fs, inode, (offset + sizeof(entry)), namelen, (uint8_t*)entry_name);

            if (error) {
                return error;
            }

            if (memcmp(entry_name, name, namelen) == 0) {
                /* We have a match! */
                return ext2_get_inode_by_idx(fs, entry.inode, inode);
            }
        }

        offset += entry.rec_len;
    }

    return "file not found";
}

const char *ext2_get_inode_by_path(
    ext2_fs_t *fs,
    const char **path_array,
    ext2_inode_t *inode
) {
    const char *error = NULL;
    
    /* Start with the root node. */
    error = ext2_get_inode_by_idx(fs, 2, inode);

    if (error) {
        return error;
    }

    /* Then search until we run out of path components. */
    const char **path_i = path_array;
    while (*path_i) {
        error = search_directory(fs, inode, *path_i);

        if (error) {
            return error;
        }

        path_i++;
    }

    return NULL;
}

/* Tail call recursive helper function for find_offset_on_disk(). */
static const char *indirect_lookup_recursive(
    ext2_fs_t *fs,
    uint32_t   index,
    uint32_t   next_block,
    uint8_t    indirections_left,
    uint32_t   offset_in_block,
    uint64_t  *disk_offset
) {
    if (next_block == 0) {
        /* Sparse file. */
        *disk_offset = 0;
        return NULL;
    }

    if (indirections_left == 0) {
        /* End of recursion. We have our answer. */
        *disk_offset = (((uint64_t)next_block) << fs->log2_block_size) + offset_in_block;
        return NULL;
    }

    /* Calculate the location of the next block address, and load it from disk. */
    uint32_t indirect_table_size = fs->block_size / 4;
    uint32_t divider = 1;
    uint8_t i = 0;
    for (i=1; i<indirections_left; i++) {
        divider *= indirect_table_size;
    }
    uint32_t local_index = (index / divider) % indirect_table_size;
    uint64_t offset = (((uint64_t)next_block) << fs->log2_block_size) + (4 * local_index);

    const char *error = cached_read(fs, offset, sizeof(next_block), (uint8_t*)&next_block);
    
    if (error) {
        return error;
    }

    return indirect_lookup_recursive(fs, index, next_block, (indirections_left-1), offset_in_block, disk_offset);
}

/* Find where a given offset in a file (inode) is on the disk. */
static const char *find_offset_on_disk(
    ext2_fs_t    *fs,
    ext2_inode_t *inode,
    uint32_t      file_offset,
    uint64_t     *disk_offset
) {
    /* Ext2 stores the first 12 block addresses directly in the inode.
     * The remaining blocks are "indirect blocks" that require 1, 2, or 3 additional table walks.
     * So, first we have to figure out how many levels of indirection we have to deal with for this block.
     */
    uint32_t file_block      = file_offset >> fs->log2_block_size;
    uint32_t offset_in_block = file_offset & fs->block_offset_mask;

    /* Build table of where the breaks are between the indirections. */
    uint32_t indirect_table_size   = fs->block_size / 4;
    uint32_t indirection_breaks[4] = { 12 };
    indirection_breaks[1] =  indirect_table_size                                          + indirection_breaks[0];
    indirection_breaks[2] = (indirect_table_size*indirect_table_size)                     + indirection_breaks[1];
    indirection_breaks[3] = (indirect_table_size*indirect_table_size*indirect_table_size) + indirection_breaks[2];

    /* Find where we are in this table. */
    uint8_t indirections = 0;
    for (indirections=0; indirections < 4; indirections++) {
        if (file_block < indirection_breaks[indirections]) {
            /* Found it! */
            break;
        }
    }

    if (indirections > 3) {
        /* Reaching this part of the file would need 4 levels of indirection. */
        return "impossible file offset";
    }

    if (indirections == 0) {
        /* Direct access. The block number is in the inode. */
        uint64_t disk_block = inode->block[file_block];
        if (disk_block == 0) {
            /* Sparse file. */
            *disk_offset = 0;
            return NULL;
        }
        *disk_offset = (disk_block << fs->log2_block_size) + offset_in_block;
        return NULL;
    }

    /* If we got here, we have to handle indirect access.
     * Figure out the index to use for the lookup, and then handle it using tail call recursion.
     */
    uint32_t block_in_indirect = file_block - indirection_breaks[indirections-1];
    return indirect_lookup_recursive(
        fs,
        block_in_indirect,
        inode->block[11+indirections],
        indirections,
        offset_in_block,
        disk_offset
    );
}

/* Find the largest contigious chunk of a file (inode) that we can read from an offset.
 * Files can be scattered on the disk. The simple approach would be to read the file one block at a time, but that
 * would be suboptimal if the file is not fragmanted.
 */
const char *find_next_contigious_chunk(
    ext2_fs_t    *fs,
    ext2_inode_t *inode,
    uint32_t      file_offset,
    uint32_t      count,
    uint64_t     *disk_offset,
    uint32_t     *disk_count
) {
    const char *error = NULL;
    
    /* Find the first offset. */
    error = find_offset_on_disk(fs, inode, file_offset, disk_offset);

    if (error) {
        return error;
    }

    *disk_count = fs->block_size - (file_offset & fs->block_offset_mask);
    if (*disk_count > count) {
        *disk_count = count;
    }
    count -= *disk_count;

    if (*disk_offset == 0) {
        /* Spare file. */
        return NULL;
    }

    /* At this point, we have the disk offset and count from the first block, but we have not tried to merge it with
     * the next block yet. To do that, we have to keep getting the next block until we either run out of bytes to read
     * or find that the file is fragmented.
     */
    while(count != 0) {
        uint64_t expected_offset = *disk_offset + *disk_count;
        uint64_t actual_offset = 0;
        error = find_offset_on_disk(fs, inode, (file_offset+*disk_count), &actual_offset);
        
        if (error) {
            return error;
        }

        if (actual_offset == expected_offset) {
            /* We can merge these! */
            if (count >= fs->block_size) {
                *disk_count += fs->block_size;
                count       -= fs->block_size;
            } else {
                *disk_count += count;
                count        = 0;
            }
        } else {
            /* The file is fragmented */
            break;
        }
    }

    return NULL;
}

const char *ext2_read(
    ext2_fs_t *fs,
    ext2_inode_t *inode,
    uint32_t offset,
    uint32_t count,
    uint8_t *buffer
) {
    const char *error = NULL;

    /* Check if the file is large enough for the requested read. */
    if (inode->size < (offset + count)) {
        return "file is too small";
    }

    /* Read each contiguous chunk. */
    while (count) {
        uint64_t disk_offset = 0;
        uint32_t disk_count  = 0;
        
        error = find_next_contigious_chunk(fs, inode, offset, count, &disk_offset, &disk_count);

        if (error) {
            return error;
        }

        if (disk_offset == 0) {
            /* Sparse file. */
            memset(buffer, 0, disk_count);
        } else {
            error = cached_read(fs, disk_offset, disk_count, buffer);

            if (error) {
                return error;
            }
        }

        count  -= disk_count;
        offset += disk_count;
        buffer += disk_count;
    }

    return NULL;
}

/* Cached access layer.
 *
 * This implements a direct mapped cache of disk blocks.
 * The cache is only used when partial block reads are requested. Block sized naturally aligned accesses hit the actual
 * storage device without checking the cache. The purpose of the cache is to speed up the internal operations of the
 * filesystem. For example, to avoid reading the same block over and over again when individual directory entries are
 * accessed.
 */

/* This function is used by cached_read to handle partial block reads. */
static const char *cached_read_subblock(
    ext2_fs_t *fs,
    uint32_t block,
    uint32_t offset,
    uint32_t count,
    uint8_t *buffer
) {
    /* Figure out which cache block would hold the block. */
    uint32_t cache_line = block % fs->cache_blocks_count;
    uint8_t *cache = ((uint8_t*)fs->cache_blocks) + (cache_line << fs->log2_block_size);

    /* Is this a cache hit? */
    if (fs->cache_tags[cache_line] == block) {
        /* Yes! Just update the counter. */
        fs->cache_hits++;
    } else {
        /* No! We have to read the requested block. */
        fs->cache_misses++;
        
        uint32_t sectors_in_block = fs->block_size/512;

        const char *error = fs->disk_access(fs->context, (block*sectors_in_block), sectors_in_block, cache);
        
        if (error) {
            return error;
        }

        fs->cache_tags[cache_line] = block;
    }

    /* At this point, we know we have a cache hit. */

    /* Just copy the data and we are done. */
    memcpy(buffer, (cache+offset), count);
    return NULL;
}

static const char *cached_read(
    ext2_fs_t *fs,
    uint64_t offset,
    uint64_t count,
    uint8_t *buffer
) {
    /* Is the start address well aligned? */
    if ((offset & fs->block_offset_mask) != 0) {
        /* It's not. Use the cache to handle the unaligned portion. */
        
        /* Figure out which block we have to read, and how much of it. */
        uint32_t block        = offset >> fs->log2_block_size;
        uint32_t block_offset = offset & fs->block_offset_mask;
        uint32_t block_count  = fs->block_size - block_offset;
        if (block_count > count) {
            block_count = count;
        }

        /* Perform the cached access. */
        const char *error = cached_read_subblock(
            fs,
            block,
            block_offset,
            block_count,
            buffer
        );

        if (error) {
            return error;
        }

        offset += block_count;
        count  -= block_count;
        buffer += block_count;
    }

    /* At this point, we have an aligned offset. */

    /* Do we have at least another block to read? */
    if (count > fs->block_size) {
        /* Yes, read it directly from the disk. */
        
        /* Figure out how much we can read directly. */
        uint32_t first_block      = offset >> fs->log2_block_size;
        uint32_t blocks_left_over = count >> fs->log2_block_size;
        uint32_t sectors_in_block = fs->block_size/512;

        const char *error = fs->disk_access(
            fs->context,
            (first_block * sectors_in_block),
            (blocks_left_over * sectors_in_block),
            buffer
        );

        if (error) {
            return error;
        }

        offset += blocks_left_over << fs->log2_block_size;
        count  -= blocks_left_over << fs->log2_block_size;
        buffer += blocks_left_over << fs->log2_block_size;
    }

    /* Is there still data leftover to read? */
    if (count != 0) {
        /* Yes, which means that the data is less than a whole block. Use the cache to read this. */
        uint32_t block = offset >> fs->log2_block_size;

        /* Perform the cached access. */
        const char *error = cached_read_subblock(
            fs,
            block,
            0,
            count,
            buffer
        );
        
        if (error) {
            return error;
        }
    }

    /* We read everything! */
    return NULL;
}