/* A G0B1-shaped memory map for the emulator: code in flash, state in RAM. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   : ORIGIN = 0x20000000, LENGTH = 144K
}
ENTRY(fixq_bench_init)
EXTERN(fixq_bench_init fixq_bench_tick)
SECTIONS
{
  .text : { KEEP(*(.text.fixq_bench_*)) *(.text .text.*) } > FLASH
  .rodata : { *(.rodata .rodata.*) } > FLASH
  .data : { *(.data .data.*) } > RAM AT > FLASH
  .bss (NOLOAD) : { *(.bss .bss.*) *(COMMON) } > RAM
  /DISCARD/ : { *(.ARM.exidx .ARM.exidx.* .ARM.extab.*) }
}
