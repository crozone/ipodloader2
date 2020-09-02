/*
 *  libgcc.a(unwind-arm.o) is built with -fexceptions and linked by the GCC compiler.
 *  However, we don't link against libgcc_eh.a because we are providing
 *  our own baremetal libc implementation.
 *
 *  This causes errors in unwind-arm-common like:
 *
 *  undefined reference to `__exidx_start'
 *  undefined reference to `__exidx_end'
 *  undefined reference to `abort'
 *
 *  Solution:
 *
 *  Add dummy definitions for these unwind routines so that linker errors don't occur.
 *  These routines should never actually be called.
 */

void
__aeabi_unwind_cpp_pr0 (void)
{
}

void
__aeabi_unwind_cpp_pr1 (void)
{
}

void
__aeabi_unwind_cpp_pr2 (void)
{
}
