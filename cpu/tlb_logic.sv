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

/* Combinational logic for a single TLB.
 *
 * The MMU has many of these units.
 */
`ifndef _TLB_LOGIC_SV
`define _TLB_LOGIC_SV

`include "common.sv"

module TLBLogic(
    /* Virtual page number of the entry. */
    input  [`VLEN-1:12]     vpn,
    /* Size of mapped page. */
    input  mmu::psize_t     psize,
    /* Page table entry. */
    input  [31:0]           pte,

    /* Set if SUM bit is set.
     * If set, user pages are accessesible in supervisor mode.
     */
    input                   sum,
    /* Set if executing in user mode. */
    input                   u_mode,

    /* Virtual address to translate. */
    input  [`VLEN-1:0]      vaddr,
    /* Set if read access. */
    input                   access_r,
    /* Set if write access. */
    input                   access_w,
    /* Set if instruction access. */
    input                   access_x,
    /* Match results. */
    output reg              match,
    /* Address matches, but the type of access is not allowed. */
    output                  fault,
    /* Fault can be resolved by setting dirty bit.
     * This must be ignored if this is not a write access or if there is no fault.
     */
    output                  fault_dns,
    /* Translated physical address. */
    output reg [`PLEN-1:0]  paddr
);

wire [`VLEN-1:12] vpn_4k = vpn[`VLEN-1:12];
wire [`VLEN-1:21] vpn_2m = vpn[`VLEN-1:21];
wire [`VLEN-1:30] vpn_1g = vpn[`VLEN-1:30];
wire [`VLEN-1:12] vpn_access_4k = vaddr[`VLEN-1:12];
wire [`VLEN-1:21] vpn_access_2m = vaddr[`VLEN-1:21];
wire [`VLEN-1:30] vpn_access_1g = vaddr[`VLEN-1:30];
wire [11:0] offset_4k = vaddr[11: 0];
wire [20:0] offset_2m = vaddr[20: 0];
wire [29:0] offset_1g = vaddr[29: 0];


/* Decode the fields of the PTE.
 * Note: we use the Sv32 format to implement Sv39.
 * (rvpriv.4.3.1)
 */

wire [`PLEN-1:12] ppn_4k = pte[30:10];
wire [`PLEN-1:21] ppn_2m = pte[30:19];
wire [`PLEN-1:30] ppn_1g = pte[30:28];

/* RSW field is reserved for software. */

/* D - Dirty. */
wire d = pte[7];

/* A - Accessed field is managed by software. */

/* G - Global. */
wire g = pte[5];

/* U - User page. */
wire u = pte[4];

/* X - eXecutable. */
wire x = pte[3];

/* W - Writable. */
wire w = pte[2];

/* R - Readable. */
wire r = pte[1];

/* V - Valid. */
wire v = pte[0];

/* Decode XWR bits.
 * (rvpriv.Table 4.4)
 */
reg readable;
reg writeable;
reg executable;
always_comb begin
    case ({x, w, r})
        3'b000: begin
            /* Pointer to next level of page table. */
            readable   = 0;
            writeable  = 0;
            executable = 0;
        end 
        3'b001: begin
            /* Read-only page. */
            readable   = 1;
            writeable  = 0;
            executable = 0;
        end
        3'b010, 3'b110: begin
            /* Reserved for future use. */
            readable   = 0;
            writeable  = 0;
            executable = 0;
        end
        3'b011: begin
            /* Read-write page. */
            readable   = 1;
            writeable  = 1;
            executable = 0;
        end
        3'b100: begin
            /* Execute-only page. */
            readable   = 0;
            writeable  = 0;
            executable = 1;
        end
        3'b101: begin
            /* Read-execute page. */
            readable   = 1;
            writeable  = 0;
            executable = 1;
        end
        3'b111: begin
            /* Read-write-execute page. */
            readable   = 1;
            writeable  = 1;
            executable = 1;
        end
    endcase
end

/* This is a match if the address matches and the entry is valid.
 * This is where we account for the large page size for address matching.
 */
always_comb begin
    automatic reg address_match;
    case (psize)
        mmu::PSIZE_4KB:     address_match = (vpn_4k == vpn_access_4k);
        mmu::PSIZE_2MB:     address_match = (vpn_2m == vpn_access_2m);
        mmu::PSIZE_1GB:     address_match = (vpn_1g == vpn_access_1g);
        
        default:            address_match = 1'bx;
    endcase

    match = address_match && v;
end

/* Generate fault if there is a match, but the type of access is not allowed.
 * Note that if the page is not dirty, we'll not allow writing. This way, the dirty bit can be managed by machine mode.
 */
wire fault_r     = (!readable       ) && access_r;
wire fault_w     = (!writeable || !d) && access_w;
wire fault_x     = (!executable     ) && access_x;
/* Generate a fault if the context matches, but the PTE does not allow access from the context. */
wire fault_c     = !((u_mode == u) || (!u_mode && sum));
assign fault     = match && (fault_r || fault_w || fault_x || fault_c);
assign fault_dns = writeable && !fault_c;

/* Output translated addresses.
 * The addresses are returned even if there is no match or there is a fault, but they must not be used.
 */
always_comb begin
    case (psize)
        mmu::PSIZE_4KB:     paddr = {ppn_4k, offset_4k};
        mmu::PSIZE_2MB:     paddr = {ppn_2m, offset_2m};
        mmu::PSIZE_1GB:     paddr = {ppn_1g, offset_1g};

        default:            paddr = 33'hxxxxxxxxx;
    endcase
end

endmodule

`endif /* _TLB_LOGIC_SV */
