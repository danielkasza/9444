/* Copyright (c) 2019, 2020, Daniel Kasza
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

/* This file is intended to be included by another assembly file. */

    /* Swap stack pointer and save 3 registers for our own use. */
    csrrw   sp, mscratch, sp
    sd      a0,  -8(sp)         // a0 will be used as the pte pointer
    sd      a1, -16(sp)         // a1 will be used for the pte
    sd      a2, -24(sp)         // a2 will be used as a temporary for branching
    
    /* Load L1 PTE. */
    csrr    a0, 0xFC0           // pte_ptr = ml1pteaddr
    ld      a1, 0(a0)           // pte = *pte_ptr
    andi    a2, a1, 1           // pte & V
    beqz    a2, 4f              // invalid entry -> load useless entry
    /* We have a valid L1 PTE, but is it a leaf? */
    andi    a2, a1, 0xE         // pte & XWR
    bnez    a2, 4f              // leaf pte -> load entry

    /* Extract base address of L2 page table. */
    srli    a1, a1, 10          // pte = pte >> 10 (discard RSW, D, A, G, U, X, W, R, V)
    slli    a1, a1, 12          // pte = pte << 12 (shift PPN bits in correct position, assuming reserved bits are not used)

    /* Load L2 PTE. */
    csrr    a0, 0xFC1           // ml2pteoff
    add     a0, a0, a1          // calculate new pte_ptr
    ld      a1, 0(a0)           // pte = *pte_ptr
    andi    a2, a1, 1           // pte & V
    beqz    a2, 2f              // invalid entry -> load useless entry
    /* We have a valid L2 PTE, but is it a leaf? */
    andi    a2, a1, 0xE         // pte & XWR
    bnez    a2, 5f              // leaf pte -> load entry

    /* Extract base address of L3 page table. */
    srli    a1, a1, 10          // pte = pte >> 10 (discard RSW, D, A, G, U, X, W, R, V)
    slli    a1, a1, 12          // pte = pte << 12 (shift PPN bits in correct position, assuming reserved bits are not used)

    /* Load L3 PTE. */
    csrr    a0, 0xFC2           // ml3pteoff
    add     a0, a0, a1          // calculate new pte_ptr
    ld      a1, 0(a0)           // pte = *pte_ptr
    andi    a2, a1, 1           // pte & V
    beqz    a2, 3f              // invalid entry -> load useless entry

    /* We do not check if the entry is a leaf.
     * This saves a couple of cycles, but it is not strictly compliant.
     * If the kernel expects that it can do a non-leaf valid entry at this level, things are not going to work anyway.
     */

    /* Load 4KB leaf entry. */
#ifdef S_TLB_MISS_HANDLER
    andi    a2, a1, 0x4         // pte & W
    beqz    a2, 1f              // not writeable -> do not set D
    ori     a1, a1, 0x80        // pte |= D (dirty)
1:
#endif
    ori     a1, a1, 0x40        // pte |= A (accessed)
    sd      a1, 0(a0)           // *pte_ptr = pte (writeback)
    csrw   0x7c0, a1            // load 4KB PTE

0:  /* Restore original context. */
    ld      a0,  -8(sp)
    ld      a1, -16(sp)
    ld      a2, -24(sp)
    csrrw   sp, mscratch, sp
    mret

3:  /* Load useless 4KB entry. */
    li      a1, 1               // pte = dummy entry
    csrw    0x7c0, a1           // load 4KB PTE
    j       0b
4:  /* Load useless 1GB entry. */
    li      a1, 1               // pte = dummy entry
    csrw    0x7c2, a1           // load 1GB PTE
    j       0b
2:  /* Load useless 2MB entry. */
    li      a1, 1               // pte = dummy entry
    csrw    0x7c1, a1           // load 2MB PTE
    j       0b

4:  /* Load 1GB leaf entry. */
#ifdef S_TLB_MISS_HANDLER
    andi    a2, a1, 0x4         // pte & W
    beqz    a2, 1f              // not writeable -> do not set D
    ori     a1, a1, 0x80        // pte |= D (dirty)
1:
#endif
    ori     a1, a1, 0x40        // pte |= A (accessed)
    sd      a1, 0(a0)           // *pte_ptr = pte (writeback)
    csrw   0x7c2, a1            // load 1GB PTE
    j       0b

5:  /* Load 2MB leaf entry. */
#ifdef S_TLB_MISS_HANDLER
    andi    a2, a1, 0x4         // pte & W
    beqz    a2, 1f              // not writeable -> do not set D
    ori     a1, a1, 0x80        // pte |= D (dirty)
1:
#endif
    ori     a1, a1, 0x40        // pte |= A (accessed)
    sd      a1, 0(a0)           // *pte_ptr = pte (writeback)
    csrw   0x7c1, a1            // load 2MB PTE
    j       0b
