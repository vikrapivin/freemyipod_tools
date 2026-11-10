ENTRY(__start)
OUTPUT_FORMAT(elf32-littlearm)
OUTPUT_ARCH(arm)
STARTUP(build/ipodnano4g/target/ipodnano4g/crt0.o)

MEMORY
{
    INIT : ORIGIN = 0x08000000, LENGTH = 0x01f80000
    SRAM : ORIGIN = 0x22000000, LENGTH = 0x00030000
    SDRAM : ORIGIN = 0x09f80000, LENGTH = 0x00054000   /* .text/.rodata/.data/.bss, up to the LCD fb */
    /* FTL mount tables. Not in the DFU image. A stage-2 load at this address replaces them. */
    FTLRAM : ORIGIN = 0x09000000, LENGTH = 0x00080000
    LCD  : ORIGIN = 0x09FD4000, LENGTH = 0x0002C000
}

SECTIONS
{
    .inithead :
    {
        _initheadstart = .;
        _poolstart = .;
	*(.inithead*)
        . = ALIGN(0x4);
        _initheadend = .;
    } > INIT

    .sram :
    {
        _sramstart = .;
        KEEP(*(.intvect))
        *(.intvect)
        *(.icode*)
        *(.irodata*)
        _sram_code_end = .;
        *(.idata*)
        . = ALIGN(0x4);
        _sramend = .;
    } > SRAM AT> INIT
    _sramsource = LOADADDR(.sram);

    /* 0x09000000 is stage 2. It is loaded into SDRAM and executed when
       needed. It is not part of the DFU image. The heap stops here so a
       later load is not allocator memory. */
    _poolend = 0x09000000;

    .sdram :
    {
        _sdramstart = .;
        *(.text*)
        . = ALIGN(0x4);
        *(.rodata*)
        . = ALIGN(0x4);
        _sdram_code_end = .;
        *(.data*)
        . = ALIGN(0x4);
        _sdramend = .;
    } > SDRAM AT> INIT
    _sdramsource = LOADADDR(.sdram);

    .init :
    {
        _initstart = .;
        *(.initcode*)
        *(.initrodata*)
        *(.initdata*)
        . = ALIGN(0x4);
        _initend = .;
    } > INIT

    .inittail :
    {
        _inittailstart = .;
        *(.inittail*)
        . = ALIGN(0x4);
        _inittailend = .;
    } > INIT

    .ibss (NOLOAD) :
    {
        _ibssstart = .;
        *(.ibss*)
        . = ALIGN(0x4);
        _abortstackstart = .;
        . += 0x400;
        _abortstackend = .;
        _irqstackstart = .;
        . += 0x400;
        _irqstackend = .;
        *(.stack*)
        . = ALIGN(0x4);
        _ibssend = .;
    } > SRAM

    .ftlstate (NOLOAD) :
    {
        *ftl-nano5g.o*(.bss*)
        *ftl-nano5g.o*(COMMON)
    } > FTLRAM

    .bss (NOLOAD) :
    {
        _bssstart = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(0x4);
        _bssend = .;
    } > SDRAM

    .lcdfb (NOLOAD) :
    {
        _lcdfbstart = .;
        *(.lcdfb*)
        . = ALIGN(0x4);
        _lcdfbend = .;
    } > LCD

    /DISCARD/ :
    {
        *(.eh_frame)
    }

    /* SRAM-BUDGET GUARD (2026-09-24 boot regression): the SELF-INIT bootstub
       (bootstub.S block0_constpool R11) puts its first-level MMU page table at
       0x2202C000 and that table remains the ACTIVE TTBR for the whole run (the
       payload crt0 never rewrites CP15 c2).  .ibss (SRAM bss) MUST end below
       0x2202C000 or crt0's clearibss zeroes live page-table entries and the
       image faults back to DFU.  Fail the link loudly instead of silently. */
    ASSERT(_ibssend <= 0x2202C000, ".ibss overflowed into the 0x2202C000 bootstub MMU page table (SRAM budget) — move a .ibss/.stack symbol to DRAM .bss")

    /* framebuffer must fit its pinned region and stay clear of SDRAM end */
    ASSERT(_lcdfbend <= 0x0A000000, ".lcdfb (LCD framebuffer) overflowed past the 0x0A000000 SDRAM end")

}
