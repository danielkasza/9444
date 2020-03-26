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

#include <stdint.h>

#pragma pack(push)

/* On-disk layout and structures of Ext2 filesystem.
 *
 * Ext2 uses little-endian byte order. It is assumed that this file will be used with little-endian machines.
 * Only revision 0 of the filesystem is supported.
 */

/* Superblock
 *
 * The superblock structure provides general information about the layout and configuration of the filesystem.
 * There are two copies. The first copy is at byte 1024, and the second copy is in block group 1.
 */
typedef struct {
    /* Total number of inodes. */
    uint32_t inodes_count;
    /* Total number of blocks. */
    uint32_t blocks_count;
    /* Number of blocks reserved for root. */
    uint32_t r_blocks_count;
    /* Number of free blocks. */
    uint32_t free_blocks_count;
    /* Number of free inodes. */
    uint32_t free_inodes_count;
    /* ID of the first data block.
     * This is the block that contains the superblock.
     */
    uint32_t first_data_block;
    /* Used to compute the block size.
     * block_size = 1024 << log_block_size
     */
    uint32_t log_block_size;
    /* Used to compute the fragment size.
     * fragment_size = 1024 << log_frag_size
     */
    uint32_t log_frag_size;
    /* Number of blocks per group. */
    uint32_t blocks_per_group;
    /* Number of fragments per group. */
    uint32_t frags_per_group;
    /* Number of inodes per group. */
    uint32_t inodes_per_group;
    /* POSIX timestamp of last mount. */
    uint32_t mtime;
    /* POSIX timestamp of last write. */
    uint32_t wtime;
    /* Number of times the filesystem was mounted since a full verification. */
    uint16_t mnt_count;
    /* Number of times the filesystem may be mounted before a full verification. */
    uint16_t max_mnt_count;
    /* Magic number used to identify Ext2 filesystem. */
    uint16_t magic;
#define EXT2_SUPERBLOCK_MAGIC 0xEF53
    /* State of the filesystem. */
    uint16_t state;
    /* This value is set after the filesystem is unmounted. */
#define EXT2_SUPERBLOCK_STATE_VALID                 1
    /* This value is set after the filesystem is mounted. */
#define EXT2_SUPERBLOCK_STATE_ERROR                 2
    /* Specifies the action to take if an error is detected. */
    uint16_t errors;
#define EXT2_SUPERBLOCK_ERRORS_CONTINUE             1
#define EXT2_SUPERBLOCK_ERRORS_READONLY             2
#define EXT2_SUPERBLOCK_ERRORS_PANIC                3
    /* Minor revision level. */
    uint16_t minor_rev_level;
    /* POSIX timestamp of last full verification. */
    uint32_t lastcheck;
    /* Maximum POSIX time before full verification. */
    uint32_t checkinterval;
    /* Operating system that created the filesystem. */
    uint32_t creator_os;
#define EXT2_SUPERBLOCK_CREATOR_OS_LINUX            0
#define EXT2_SUPERBLOCK_CREATOR_OS_HURD             1
#define EXT2_SUPERBLOCK_CREATOR_OS_MASIX            2
#define EXT2_SUPERBLOCK_CREATOR_OS_FREEBSD          3
#define EXT2_SUPERBLOCK_CREATOR_OS_LITES            5
    /* Major revision level.
     * Only revision 0 is supported with this header!
     */
    uint32_t rev_level;
#define EXT2_SUPERBLOCK_REV_LEVEL_EXPECTED          0
    /* User ID for reserved blocks. */
    uint16_t def_resuid;
    /* Group ID for reserved blocks. */
    uint16_t def_resgid;
    /* Extensions that use the remaining fields are not supported. */
} ext2_superblock_t;
/* Total size of the superblock, including reserved fields. */
#define EXT2_SUPERBLOCK_SIZE                        1024

/* Block group descriptor table entry.
 *
 * There is one of these stored after every copy of the superblock.
 * The table may use multiple blocks. There is an entry like this for every block group.
 */
typedef struct {
    /* ID of first block of block bitmap. */
    uint32_t block_bitmap;
    /* ID of first block of inode bitmap. */
    uint32_t inode_bitmap;
    /* ID of first block of inode table. */
    uint32_t inode_table;
    /* Number of free blocks in group. */
    uint16_t free_blocks_count;
    /* Number of free inodes in group. */
    uint16_t free_inodes_count;
    /* Number of inodes used for directories in group. */
    uint16_t used_dirs_count;
    uint16_t pad;
    uint16_t reserved[6];
} ext2_block_group_descriptor_t;

/* Inode table entry. */
typedef struct {
    /* Type of file and access rights. */
    uint16_t mode;
#define EXT2_INODE_MODE_IFMASK                      0xF000
#define EXT2_INODE_MODE_IFSOCK                      0xC000
#define EXT2_INODE_MODE_IFLNK                       0xA000
#define EXT2_INODE_MODE_IFREG                       0x8000
#define EXT2_INODE_MODE_IFBLK                       0x6000
#define EXT2_INODE_MODE_IFDIR                       0x4000
#define EXT2_INODE_MODE_IFCHR                       0x2000
#define EXT2_INODE_MODE_IFIFO                       0x1000
#define EXT2_INODE_MODE_ISUID                       0x0800
#define EXT2_INODE_MODE_ISGID                       0x0400
#define EXT2_INODE_MODE_ISVTX                       0x0200
#define EXT2_INODE_MODE_IRUSR                       0x0100
#define EXT2_INODE_MODE_IWUSR                       0x0080
#define EXT2_INODE_MODE_IXUSR                       0x0040
#define EXT2_INODE_MODE_IRGRP                       0x0020
#define EXT2_INODE_MODE_IWGRP                       0x0010
#define EXT2_INODE_MODE_IXGRP                       0x0008
#define EXT2_INODE_MODE_IROTH                       0x0004
#define EXT2_INODE_MODE_IWOTH                       0x0002
#define EXT2_INODE_MODE_IXOTH                       0x0001
    /* User ID. */
    uint16_t uid;
    /* Size of the file. */
    uint32_t size;
    /* POSIX timestamp of last access. */
    uint32_t atime;
    /* POSIX timestamp of creation. */
    uint32_t ctime;
    /* POSIX timestamp of last modification. */
    uint32_t mtime;
    /* POSIX timestamp of last deletion. */
    uint32_t dtime;
    /* Group ID. */
    uint16_t gid;
    /* Number of hard links to this file. */
    uint16_t links_count;
    /* Number of 512B blocks reserved for this file.
     * Note: this is counted using 512B blocks, not filesystem blocks!
     */
    uint32_t blocks;
    /* Flags for special uses.
     * This library will not support any of these, so just make sure this is 0.
     */
    uint32_t flags;
    /* OS dependent value. */
    uint32_t osd1;
    /* IDs of blocks containing the file data.
     * The first 12 blocks are direct blocks.
     * block[12] is an indirect block.
     * block[13] is a doubly indirect block.
     * block[14] is a triply indirect block.
     * A value of 0 terminates the file.
     */
    uint32_t block[15];
    /* File version. */
    uint32_t generation;
    /* Should be 0 in revision 0. */
    uint32_t file_acl;
    /* Should be 0 in revision 0. */
    uint32_t dir_acl;
    /* Should be 0. */
    uint32_t faddr;
    /* OS dependent values. */
    uint32_t osd2[3];
} ext2_inode_t;

/* Inode lookup helper functions.
 * Inode numbers start at 1.
 * These utility functions can be used to find the group and local inode index of an inode.
 */
static inline uint32_t ext2_get_inode_group(uint32_t inode_idx, uint32_t inodes_per_group) {
    return (inode_idx - 1) / inodes_per_group;
}
static inline uint32_t ext2_get_inode_local_idx(uint32_t inode_idx, uint32_t inodes_per_group) {
    return (inode_idx - 1) % inodes_per_group;
}

/* Directory entry structure. */
typedef struct {
    /* Inode number of the file. */
    uint32_t inode;
    /* Size of this entry. */
    uint16_t rec_len;
    /* Length of name. */
    uint8_t  namelen;
    /* Not used in revision 0. */
    uint8_t  file_type;
    /* Name of this file. Not '\0' terminated! */
    char     name[/* namelen */];
} ext2_dirent_t;

#pragma pack(pop)