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

/* 9444 MMU.
 *
 * The MMU contains multiple TLBs that can be configured through a single interface.
 *
 * It is expected that the software will not configure two TLBs with conflicting configurations. If this happens anyway,
 * no error will be reported, and the TLB with the lowest index will take precedent.
 *
 * Large pages (2MB and 1GB) are handled using a few (up to 32) TLB entries. These are fully associative.
 * Normal (4KB) pages are handled by an array of entries that share the same TLB logic. These form a set-associative
 * translation cache.
 * This organization was designed to reduce the routing delay through the MMU. Note that the MMU has to provide
 * translations combinationally. Using a large fully associative TLB array had very high routing delays.
 */
`ifndef _MMU_SV
`define _MMU_SV

`include "tlb_logic.sv"

module MMU(
    input                   clock,

    /* Set if SUM bit is set.
     * If set, user pages are accessible in supervisor mode.
     */
    input                   sum,
    /* Set if executing in user mode. */
    input                   u_mode,
    /* Set if address translation is enabled. */
    input                   enabled,

    /* Invalidate TLB entries. */
    input                   invalidate,
    /* If set, invalidate only the entries that match the address. */
    input                   selective_invalidate,
    input [`VLEN-1:0]       selective_invalidate_vaddr,

    /* Entry being modified or read. */
    input  [ 4:0]           entry,

    /** PORT FOR WRITING TLB ENTRIES. **/
    /* Virtual page number of entry being written. */
    input  [`VLEN-1:12]     vpn_in,
    input  mmu::psize_t     psize_in,
    /* Page table entry to write. */
    input  [31:0]           pte_in,
    /* If set, the entry will be saved. */
    input                   pte_write,

    /** ACCESS PORT. **/
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
    /* Index of TLB entry that matched, if match.
     * If the MMU is disabled, this will always report 0.
     *
     * This output is provided to speed up processing of data access faults.
     * It is expected that most of the faults will be caused by a write to a page that is not dirty (D). In this case,
     * the software will have to update the TLB entry. By having the hardware capture the index of the TLB entry,
     * software can avoid linear search of the TLB entries.
     */
    output reg [ 4:0]       match_entry,
    /* Address matches, but the type of access is not allowed. */
    output reg              fault,
    /* Fault can be resolved by setting dirty bit.
     * This must be ignored if this is not a write access or if there is no fault.
     */
    output reg              fault_dns,
    /* Translated physical address. */
    output reg [`PLEN-1:0]  paddr
);


/* Log 2 size of the number of large page TLBs.
 * Must be <= 5!
 */
parameter lp_tlb_log2count = 3;
localparam lp_tlb_count = (1 << lp_tlb_log2count);

/* Log 2 size of the number of normal page entry sets. */
parameter np_tlb_log2sets = 6;
localparam np_tlb_sets = (1 << np_tlb_log2sets);
/* Log 2 size of the number of normal page entry ways. */
parameter np_tlb_log2ways = 1;
localparam np_tlb_ways = (1 << np_tlb_log2ways);

/* One set of the normal page cache. */
typedef struct packed {
    /* Virtual page number of each way. */
    logic [np_tlb_ways-1:0][`VLEN-1:12+np_tlb_log2sets] vpns;
    /* Page table entries, not including valid bits. */
    logic [np_tlb_ways-1:0][31:1]                       ptes;
} np_tlb_set_t;

(* ram_style = "distributed" *) reg [$bits(np_tlb_set_t)-1:0] np_tlb_set_array[(np_tlb_sets-1):0];

/* Configured virtual page numbers for large pages. */
reg [`VLEN-1:21]                                    lp_vpns  [lp_tlb_count-1:0];
/* Configured page table entries, not including valid bits for large pages. */
reg [31:1]                  lp_ptes  [lp_tlb_count-1:0];
/* Valid bits of entries. */
reg [lp_tlb_count-1:0]                  lp_ptes_valid;
reg [ np_tlb_sets-1:0][np_tlb_ways-1:0] np_ptes_valid;
/* Page size for large pages.
 * If set, 1GB page, 2MB otherwise.
 */
reg              lp_psizes[lp_tlb_count-1:0];

/* Index of entry to write. */
wire [lp_tlb_log2count-1:0] lp_write_idx = entry[lp_tlb_log2count-1:0];
wire [ np_tlb_log2sets-1:0] np_write_set = vpn_in[12+np_tlb_log2sets-1:12];
wire [ np_tlb_log2ways-1:0] np_write_way = entry[np_tlb_log2ways-1:0];
/* Index of entry to invalidate. */
wire [ np_tlb_log2sets-1:0] np_invalidate_set = selective_invalidate_vaddr[12+np_tlb_log2sets-1:12];

/* Configuration interface. */
always @(posedge clock) begin
    automatic integer i;
    if (invalidate && selective_invalidate) begin
        /* Clear valid bit of matching entries. */

        /* For large pages, we check the top bit of the address. */
        for (i=0; i<lp_tlb_count; i++) begin
            if (lp_vpns[i][`VLEN-1] == selective_invalidate_vaddr[`VLEN-1]) begin
                lp_ptes_valid[i] <= 0;
            end
        end

        /* For normal pages, we invalidate the entries that could match the provided address. */
        np_ptes_valid[np_invalidate_set] <= 0;
    end
    else if (invalidate) begin
        /* Clear valid bit of all entries. */
        lp_ptes_valid <= 0;
        np_ptes_valid <= 0;
    end
    else if (pte_write) begin
        if (psize_in == mmu::PSIZE_4KB) begin
            automatic np_tlb_set_t np_tlb_set = np_tlb_set_array[np_write_set];
            np_tlb_set.vpns[np_write_way]              = vpn_in[`VLEN-1:12+np_tlb_log2sets];
            np_tlb_set.ptes[np_write_way]              = pte_in[31:1];
            np_tlb_set_array[np_write_set] <= np_tlb_set;
            np_ptes_valid[np_write_set][np_write_way] <= pte_in[0];
        end
        else begin
            lp_vpns[lp_write_idx]       <= vpn_in[`VLEN-1:21];
            lp_ptes[lp_write_idx]       <= pte_in[31:1];
            lp_ptes_valid[lp_write_idx] <= pte_in[0];
            lp_psizes[lp_write_idx]     <= (psize_in == mmu::PSIZE_1GB);
        end
    end
end

/* Match bits of large page TLBs. */
wire [lp_tlb_count-1:0] lp_matchs;
/* Fault bits of large page TLBs. */
wire [lp_tlb_count-1:0] lp_faults;
/* paddr of each large page TLB. */
wire [`PLEN-1:0]        lp_paddrs[lp_tlb_count-1:0];
/* Fault_dns flag of each large page TLB. */
wire [lp_tlb_count-1:0] lp_fault_dnss;

/* Large page TLB instances. */
genvar i;
for (i=0; i<lp_tlb_count; i++) begin
    TLBLogic LP_TLBLogic(
        .vpn({lp_vpns[i], 9'b0}),
        .psize(lp_psizes[i] ? mmu::PSIZE_1GB : mmu::PSIZE_2MB),
        .pte({lp_ptes[i], lp_ptes_valid[i]}),
        
        .sum(sum),
        .u_mode(u_mode),

        .vaddr(vaddr),
        .access_r(access_r),
        .access_w(access_w),
        .access_x(access_x),
        .match(lp_matchs[i]),
        .fault(lp_faults[i]),
        .fault_dns(lp_fault_dnss[i]),
        .paddr(lp_paddrs[i])
    );
end

/* Normal page TLB instances. */
wire [np_tlb_log2sets-1:0]        np_access_set_idx = vaddr[12+np_tlb_log2sets-1:12];
wire np_tlb_set_t                 np_access_set = np_tlb_set_array[np_access_set_idx];
wire [np_tlb_ways-1:0]            np_matchs;
wire [np_tlb_ways-1:0]            np_faults;
wire [np_tlb_ways-1:0]            np_fault_dnss;
wire [np_tlb_ways-1:0][`PLEN-1:0] np_paddrs;
for (i=0; i<np_tlb_ways; i++) begin
    TLBLogic NP_TLBLogic(
        .vpn({np_access_set.vpns[i], np_access_set_idx}),
        .psize(mmu::PSIZE_4KB),
        .pte({np_access_set.ptes[i], np_ptes_valid[np_access_set_idx][i]}),

        .sum(sum),
        .u_mode(u_mode),

        .vaddr(vaddr),
        .access_r(access_r),
        .access_w(access_w),
        .access_x(access_x),
        .match(np_matchs[i]),
        .fault(np_faults[i]),
        .fault_dns(np_fault_dnss[i]),
        .paddr(np_paddrs[i])
    );
end

/* Output the first matching TLB.
 * If there is no match, output a fault.
 */
always_comb begin
    automatic integer i;
    
    if (enabled) begin
        match       = 0;
        match_entry = 0;
        fault       = 0;
        fault_dns   = 0;
        paddr       = 0;
        for (i=0; i<np_tlb_ways; i++) begin
            if(np_matchs[i]) begin
                match = 1;
                match_entry = i[4:0];
                fault     = np_faults[i];
                fault_dns = np_fault_dnss[i];
                paddr     = np_paddrs[i];
                break;
            end
        end
        if (!match) for (i=0; i<lp_tlb_count; i++) begin
            if (lp_matchs[i]) begin
                match       = 1;
                match_entry = i[4:0];
                fault       = lp_faults[i];
                fault_dns   = lp_fault_dnss[i];
                paddr       = lp_paddrs[i];
                break; 
            end
        end
    end
    else begin
        /* MMU is disabled, so identity map everything. */
        match       = 1;
        match_entry = 0;
        fault       = 0;
        fault_dns   = 0;
        paddr       = vaddr[`PLEN-1:0];
    end
end

endmodule

`endif /* _MMU_SV */
