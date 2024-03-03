/*
 * startup.s - iPodLinux loader
 *
 * Copyright (c) 2003, Daniel Palffy (dpalffy (at) rainstorm.org)
 * Copyright (c) 2005, Bernard Leach <leachbj@bouncycastle.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *      Do not meddle in the affairs of Wizards, for they are subtle
 *      and quick to anger.
 *
 *      -"The Fellowship of the Ring", J.R.R Tolkien
 *
 * Here's an attempt at some information on how this all works (by TT, 30Apr06):
 *
 * This loader code got appended to the "osos" firmware file by a tool such
 * as "make_fw". The ipod's flash ROM bootloader loads the entire "osos"
 * file to memory, starting at either 0x28000000 or 0x10000000, which is
 * the iPod's SDRAM start.
 * The "osos" file's directory entry also maintains a entry address, which was
 * changed to point to this code here, which is effectively the start of
 * the "loader.bin" file (you can see this by issuing the command
 *   "arm-uclinux-elf-objdump -d loader.elf | less")
 * This loader code is built to run at address 0x40000000, though, which is
 * the start of the PP's second RAM area, called IRAM or Fast RAM.
 * Hence, this startup code here copies the loader code to 0x40000000 and
 * then runs it from there.
 */
 
/* The loader base address. Loader is build to be run from IRAM (Fast RAM) */
        .equ    LOADER_BASE, 0x40000000
 
/* Memory addresses for the PP5002 */
        .equ    PP5002_SDRAM, 0x28000000
        .equ    PP5002_PROC_ID, 0xc4000000
        .equ    PP5002_COP_CTRL, 0xcf004058
        
/* Memory addresses for the PP5020 */
        .equ    PP5020_SDRAM, 0x10000000
        .equ    PP5020_PROC_ID, 0x60000000
        .equ    PP5020_COP_CTRL, 0x60007004

/* Values for determining CPU kind (main CPU vs COP) from processor ID */
        .equ    PROC_ID_KIND_MASK, 0xff
        .equ    PROC_ID_KIND_CPU, 0x55
        .equ    PROC_ID_KIND_COP, 0xaa

/* CPU modes */
        .equ    MODE_MASK, 0x1f
        .equ    T_BIT, 0x20
        .equ    F_BIT, 0x40
        .equ    I_BIT, 0x80
        .equ    MODE_IRQ, 0x12
        .equ    MODE_SVC, 0x13
        .equ    MODE_SYS, 0x1f

/*
 * Important note:
 *  The following code fragement must not compile to more than 0x100 bytes without
 *  modifying make_fw.c to accomodate the extra room required.
 *  Therefore, part of this code is moved behind the boot_table to ensure that
 *  the boot_table starts at offset 0x100.
 */

.global _start
_start:
       /* CPU MODEL DETECTION
        *
        * We need to detect if we're running on a PP5002 or a PP5020
        * since the two CPUs have different memory mappings.
        *
        * The PP5002 SDRAM starts at 0x28000000 (PP5002_SDRAM)
        * The PP5020 SDRAM starts at 0x10000000 (PP5020_SDRAM)
        *
        * So, if the current PC is within the 0x28000000 address space,
        * we're on a PP5502. Else, assume we're on a PP5020.
        *
        * PC & 0xff000000 is stored in r8.
        * The rest of the code can then check the CPU model at any time with:
        *
        * cmp     r8, #PP5002_SDRAM
        *
        * If PP5002, the Z (zero) flag is set. Else, the Z flag is cleared.
        */

        ldr     r0, =0xff000000
        and     r8, pc, r0

       /* CPU vs COP DETECTION
        *
        * Both the main CPU and COP (coprocessor) start executing from the same start address,
        * so the current thread could be running on either the CPU or COP.
        *
        * Detect whether we're running on the CPU or COP:
        *
        * The CPU Processor ID has a lower byte of 0x55.
        * The COP Processor ID has a lower byte of 0xaa.
        */

        /* Load the CPU ID into r0 */
        /* The processor ID address is different depending on the CPU model */
        cmp     r8, #PP5002_SDRAM
        moveq   r0, #PP5002_PROC_ID
        movne   r0, #PP5020_PROC_ID
        ldr     r0, [r0]
        and     r0, r0, #PROC_ID_KIND_MASK
        cmp     r0, #PROC_ID_KIND_CPU

        /*
         * NOTE: In GNU assembly, xb and xf are label extensions.
         * 1b means "search backwards for the next label that is "1".
         * 1f means "search forwards for the next label that is "1".
         */

        /* If we're on the main CPU, jump to loader memcopy */
        beq     1f

        /* We're on the COP. Go to sleep by writing to the COP control register */
        /* The COP control register address and layout is different depending on the CPU model */
        cmp     r8, #PP5002_SDRAM
        ldreq   r4, =PP5002_COP_CTRL
        moveq   r3, #0xca
        ldrne   r4, =PP5020_COP_CTRL
        movne   r3, #0x80000000
        /* Go to sleep */
        str     r3, [r4]

        /* The COP is now asleep, waiting to be woken up by the main CPU. */
        /* After waking up, jump to cop_wake_start */
        ldr     pc, =cop_wake_start
cop_wake_start:
        /* jump the COP to startup */
        ldr     r0, =startup_loc
        ldr     pc, [r0]

       /* START LOADER CODE MEMCOPY
        *
        * The loader code is built to run at address 0x40000000,
        * which is the start of the PP's second RAM area, called IRAM or Fast RAM.
        * The loader code will be copied from the text region to address 0x40000000.
        */

1:      /* get the high part of our execute address */
        ldr     r2, =0xffffff00
        and     r4, pc, r2

        /* copy the code to 0x40000000 (LOADER_BASE) */
        mov     r5, #LOADER_BASE         /* start of code */
        ldr     r6, =__data_start__     /* end of code */
        sub     r0, r6, r5      /* lenth of text */
        add     r0, r4, r0      /* r0 points to start of text */

1:      cmp     r5, r6
        ldrcc   r2, [r4], #4
        strcc   r2, [r5], #4
        bcc     1b

        ldr     pc, =start_loc  /* jump to the next instruction in 0x4000xxxx */


startup_loc:
        .word   0x0

.align 8        /* starts at 0x100 */
.global boot_table
boot_table:
        /* Here comes the bootloader table, don't move its offset
         *
         * "make_fw" patches of list of up to 5 "sub image" entries
         * into this place.
         * This was used by the older loader to locate the
         * Apple or Linux sub-image inside this loaded image.
         * (See also http://ipodlinux.org/Firmware)
         */
        .space 0x100


start_loc:      /* this code runs in Fast RAM now */

        /* copy the DATA section to Fast RAM */
        ldr     r1, =__data_start__
        ldr     r3, =__bss_start__
        cmp     r0, r1
        beq     init_bss

1:      cmp     r1, r3
        ldrcc   r2, [r0], #4
        strcc   r2, [r1], #4
        bcc     1b

init_bss:
        /* clear the BSS */
        ldr     r1, =__bss_end__
        mov     r2, #0x0

1:      cmp     r3, r1
        strcc   r2, [r3], #4
        bcc     1b

        @ set the stack pointers
        @ switch to IRQ mode and set its stack pointer to end of Fast RAM
        @ then switch back to SVC mode and set its sp to a few KBs before the sp_IRQ
        mrs     r0, spsr
        bic     r0, r0, #MODE_MASK              @ preserve I, F and T bits
        orr     r0, r0, #MODE_IRQ               @ switch to IRQ mode
        msr     spsr_c, r0
        ldr     sp, =0x40017f00
        bic     r0, r0, #MODE_MASK
        orr     r0, r0, #MODE_SVC               @ switch back to SVC mode
        msr     spsr_c, r0
        ldr     sp, =0x40017f00
        add     sp, sp, #-0x0f00                @ this is the amount we give to the IRQ stack

        /* call the loader function */
        bl      loader
        
        /* save the startup address for the COP */
        ldr     r1, =startup_loc
        str     r0, [r1]

        cmp     r8, #PP5002_SDRAM
        bne     pp5020

        /* make sure COP is sleeping */
        ldr     r4, =0xcf004050
1:      ldr     r3, [r4]
        ands    r3, r3, #0x4000
        beq     1b

        /* wake up COP */
        ldr     r4, =PP5002_COP_CTRL
        mov     r3, #0xce
        strh    r3, [r4]

        /* jump to start location */
        mov     pc, r0

pp5020:
        /* make sure COP is sleeping */
        ldr     r4, =PP5020_COP_CTRL
1:      ldr     r3, [r4]
        ands    r3, r3, #0x80000000
        beq     1b

        /* wake up COP */
        mov     r3, #0x0
        str     r3, [r4]

        /* jump to start location */
        mov     pc, r0

