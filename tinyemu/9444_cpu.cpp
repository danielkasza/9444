#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>
#include <fcntl.h>
#include <time.h>

extern "C" {
#include "cutils.h"
#include "iomem.h"
#include "riscv_cpu.h"
}

#include <verilated.h>
#include "Vcpu.h"
#ifdef VCD_TRACE
#include <verilated_vcd_c.h>
#endif

#if 0
#define TRACE_FUNC fprintf(stderr, "%s\n", __FUNCTION__)
#else
#define TRACE_FUNC
#endif

#ifdef VCD_TRACE
static VerilatedVcdC* trace;
#define START_TRACE_ON_CYCLE    0
#define TRACE_MODULE(_module_, _file_) do { (_module_)->trace(trace, 99); trace->open((_file_)); } while (0)
#define TRACE(_time_) trace->dump(_time_)
#else
#define TRACE(_time_)
#endif

#ifdef LOG_MEMORY_ACCESSES
static FILE *mem_access_log_file;
#endif

static Vcpu *cpu;
static PhysMemoryMap *mem_map = NULL;
static uint64_t cycle = 0;

static void cpu_9444_panic(void) {
#ifdef VCD_TRACE
    trace->flush();
    trace->close();
#endif
    exit(1);
}

static uint8_t byte_read(uint64_t paddr) {
    TRACE_FUNC;
    PhysMemoryRange *pmr = get_phys_mem_range(mem_map, paddr);
    
    if (!pmr) {
        fprintf(stderr, "bad byte memory read at %" PRIx64 "\n", paddr);
        cpu_9444_panic();
    }

    if (pmr->is_ram) {
        uint8_t *ptr = pmr->phys_mem + (paddr - pmr->addr);
        return *ptr;
    } else {
        uint64_t offset = paddr - pmr->addr;
        return pmr->read_func(pmr->opaque, offset, 0);
    }
}

static void byte_write(uint64_t paddr, uint8_t val) {
    TRACE_FUNC;
    PhysMemoryRange *pmr = get_phys_mem_range(mem_map, paddr);
    
    if (!pmr) {
        fprintf(stderr, "bad byte memory write at %" PRIx64 "\n", paddr);
        cpu_9444_panic();
    }

    phys_mem_set_dirty_bit(pmr, paddr - pmr->addr);

    if (pmr->is_ram) {
        uint8_t *ptr = pmr->phys_mem + (paddr - pmr->addr);
        *ptr = val; 
    } else {
        uint64_t offset = paddr - pmr->addr;
        pmr->write_func(pmr->opaque, offset, val, 0);
    }
}

static uint16_t hword_read(uint64_t paddr) {
    TRACE_FUNC;
    PhysMemoryRange *pmr = get_phys_mem_range(mem_map, paddr);
    
    if (!pmr) {
        fprintf(stderr, "bad hword memory read at %" PRIx64 "\n", paddr);
        cpu_9444_panic();
    }

    if (pmr->is_ram) {
        uint16_t *ptr = (uint16_t*)(pmr->phys_mem + (paddr - pmr->addr));
        return *ptr;
    } else {
        uint64_t offset = paddr - pmr->addr;
        return pmr->read_func(pmr->opaque, offset, 1);
    }
}

static void hword_write(uint64_t paddr, uint16_t val) {
    TRACE_FUNC;
    PhysMemoryRange *pmr = get_phys_mem_range(mem_map, paddr);
    
    if (!pmr) {
        fprintf(stderr, "bad hword memory write at %" PRIx64 "\n", paddr);
        cpu_9444_panic();
    }

    phys_mem_set_dirty_bit(pmr, paddr - pmr->addr);

    if (pmr->is_ram) {
        uint16_t *ptr = (uint16_t*)(pmr->phys_mem + (paddr - pmr->addr));
        *ptr = val; 
    } else {
        uint64_t offset = paddr - pmr->addr;
        pmr->write_func(pmr->opaque, offset, val, 1);
    }
}

static uint32_t word_read(uint64_t paddr) {
    TRACE_FUNC;
    PhysMemoryRange *pmr = get_phys_mem_range(mem_map, paddr);
    
    if (!pmr) {
        fprintf(stderr, "bad word memory read at %" PRIx64 "\n", paddr);
        cpu_9444_panic();
    }

    if (pmr->is_ram) {
        uint32_t *ptr = (uint32_t*)(pmr->phys_mem + (paddr - pmr->addr));
        return *ptr;
    } else {
        uint64_t offset = paddr - pmr->addr;
        return pmr->read_func(pmr->opaque, offset, 2);
    }
}

static void word_write(uint64_t paddr, uint32_t val) {
    TRACE_FUNC;
    PhysMemoryRange *pmr = get_phys_mem_range(mem_map, paddr);
    
    if (!pmr) {
        fprintf(stderr, "bad word memory write at %" PRIx64 "\n", paddr);
        cpu_9444_panic();
    }

    phys_mem_set_dirty_bit(pmr, paddr - pmr->addr);
    
    if (pmr->is_ram) {
        uint32_t *ptr = (uint32_t*)(pmr->phys_mem + (paddr - pmr->addr));
        *ptr = val; 
    } else {
        uint64_t offset = paddr - pmr->addr;
        pmr->write_func(pmr->opaque, offset, val, 2);
    }
}

static void dword_write(uint64_t paddr, uint64_t val) {
    TRACE_FUNC;
    
    uint32_t lower_word = val & 0xFFFFFFFFU;
    uint32_t upper_word = val >> 32;

    word_write(paddr + 0, lower_word);
    word_write(paddr + 4, upper_word);
}

static void cpu_9444_cpu_end(RISCVCPUState *s) {
    TRACE_FUNC;
    (void)s;
}

static void cpu_9444_cpu_interp(RISCVCPUState *s, int n_cycles) {
    static uint64_t mem_cycles = 0;
    static uint64_t line_cycles = 0;
    TRACE_FUNC;
    (void)s;

    while (n_cycles) {
#if 0
        static clock_t t_last = 0;
        if (cycle % 1000000 == 0) {
            clock_t t_now = clock();
            
            if (cycle != 0) {
                float mhz = (1.0 / (t_now-t_last) * CLOCKS_PER_SEC);
                fprintf(
                    stderr, "\n%uM cycles done (~%.1fMHz), %uM memory accesses, %uM line accesses\n",
                    (unsigned)(cycle/1000000), mhz, (unsigned)(mem_cycles/1000000), (unsigned)(line_cycles/1000000)
                );
            }

            t_last = t_now;
        }
#endif

#ifdef VCD_TRACE
        if (cycle == START_TRACE_ON_CYCLE) {
            TRACE_MODULE(cpu, "9444-simulator.vcd");
        }
#endif

        cpu->clock = 0;
        cpu->eval();

        cpu->clock = 1;
        cpu->eval();

        TRACE(cycle);

        uint64_t paddr = cpu->mem_paddr;
        if (cpu->mem_cycle) {
            mem_cycles++;

#ifdef LOG_MEMORY_ACCESSES
            fprintf(
                mem_access_log_file,
                "%c%" PRIx64 "\n",
                cpu->mem_access & 1 ? 'w' : 'r',
                paddr
            );
#endif

            switch(cpu->mem_access) {
                default:
                case 0:     /* BYTE_READ */
                    cpu->mem_data_in[0] = byte_read(paddr);
                    cpu->mem_data_in[1] = 0;
                    break;

                case 1:     /* BYTE_WRITE */
                    byte_write(paddr, cpu->mem_data_out);
                    break;

                case 2:     /* HWORD_READ */
                    cpu->mem_data_in[0] = hword_read(paddr);
                    cpu->mem_data_in[1] = 0;
                    break;
                
                case 3:     /* HWORD_WRITE */
                    hword_write(paddr, cpu->mem_data_out);
                    break;

                case 4:     /* WORD_READ */
                    cpu->mem_data_in[0] = word_read(paddr);
                    cpu->mem_data_in[1] = 0;
                    break;

                case 5:     /* WORD_WRITE */
                    word_write(paddr, cpu->mem_data_out);
                    break;

                case 6:    /* DWORD_READ */
                    cpu->mem_data_in[0]  = word_read(paddr + 0);
                    cpu->mem_data_in[1]  = word_read(paddr + 4);
                    break;

                case 7:    /* DWORD_WRITE */
                    dword_write(paddr, cpu->mem_data_out);
                    break;

                case 8:     /* LINE_READ */
                    line_cycles++;
                    for (unsigned i=0; i<8; i++) {
                        cpu->mem_data_in[i] = word_read(paddr+(i*4));
                    }
                    break;
            }
            cpu->mem_ack = 1;
        }

        cycle++;
        n_cycles--;
    }

#ifdef VCD_TRACE
    trace->flush();
#endif
}

static uint64_t cpu_9444_cpu_get_cycles(RISCVCPUState *s) {
    TRACE_FUNC;
    (void)s;
    return cycle;
}

/* We only care about interrupts targeting the supervisor. */
static void cpu_9444_cpu_set_mip(RISCVCPUState *s, uint32_t mask) {
    TRACE_FUNC;
    (void)s;
    if (mask & MIP_SEIP) {
        cpu->s_interrupt = 1;
    }
}

static void cpu_9444_cpu_reset_mip(RISCVCPUState *s, uint32_t mask) {
    TRACE_FUNC;
    (void)s;
    if (mask & MIP_SEIP) {
        cpu->s_interrupt = 0;
    }
}

static uint32_t cpu_9444_cpu_get_mip(RISCVCPUState *s) {
    TRACE_FUNC;
    (void)s;
    return (cpu->s_interrupt) ?  MIP_SEIP : 0;
}

static BOOL cpu_9444_cpu_get_power_down(RISCVCPUState *s) {
    TRACE_FUNC;
    (void)s;
    return false;
}

static uint32_t cpu_9444_cpu_get_misa(RISCVCPUState *s) {
    TRACE_FUNC;
    return 0b01000000000000000001000100000001;
}

static void cpu_9444_cpu_flush_tlb_write_range_ram(RISCVCPUState *s, uint8_t *ram_ptr, size_t ram_size) {
    TRACE_FUNC;
    (void)s;
}

static RISCVCPUClass cpu_9444_class = {
    .riscv_cpu_init                      = NULL,
    .riscv_cpu_end                       = &cpu_9444_cpu_end,
    .riscv_cpu_interp                    = &cpu_9444_cpu_interp,
    .riscv_cpu_get_cycles                = &cpu_9444_cpu_get_cycles,
    .riscv_cpu_set_mip                   = &cpu_9444_cpu_set_mip,
    .riscv_cpu_reset_mip                 = &cpu_9444_cpu_reset_mip,
    .riscv_cpu_get_mip                   = &cpu_9444_cpu_get_mip,
    .riscv_cpu_get_power_down            = &cpu_9444_cpu_get_power_down,
    .riscv_cpu_get_misa                  = &cpu_9444_cpu_get_misa,
    .riscv_cpu_flush_tlb_write_range_ram = &cpu_9444_cpu_flush_tlb_write_range_ram,
};

static RISCVCPUCommonState common_state = {
    .class_ptr = &cpu_9444_class,
};

RISCVCPUState *riscv_cpu_init(PhysMemoryMap *_mem_map, int max_xlen) {
    TRACE_FUNC;

    if (max_xlen != 64) {
        fprintf(stderr, "Only 64b RISC-V machines are supported!\n");
    }

#ifdef VCD_TRACE
    Verilated::traceEverOn(true);
#endif

#ifdef LOG_MEMORY_ACCESSES
    mem_access_log_file = fopen("9444-simulator.mem.txt", "w");
    if (mem_access_log_file == NULL) {
        fprintf(stderr, "Could not open memory access log!\n");
        cpu_9444_panic();
    }
#endif

    mem_map = _mem_map;
    cpu = new Vcpu;

#ifdef  VCD_TRACE
    trace = new VerilatedVcdC;
#endif

    return (RISCVCPUState*)&common_state;
}

/* Verilator requires this now. */
double sc_time_stamp() {
    return (double)cycle;
}