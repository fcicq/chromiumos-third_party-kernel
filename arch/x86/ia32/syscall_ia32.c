/* System call table for ia32 emulation. */

#include <linux/linkage.h>
#include <linux/sys.h>
#include <linux/cache.h>
#include <linux/thread_info.h> /* for ti_sys_call_ptr_t */
#include <asm/asm-offsets.h>
#include <asm/syscall.h>

#define __SYSCALL_I386(nr, sym, compat) extern asmlinkage void compat(void) ;
#include <asm/syscalls_32.h>
#undef __SYSCALL_I386

#define __SYSCALL_I386(nr, sym, compat) [nr] = compat,

extern void compat_ni_syscall(void);

const ti_sys_call_ptr_t ia32_sys_call_table[__NR_ia32_syscall_max+1] = {
	/*
	 * Smells like a compiler bug -- it doesn't work
	 * when the & below is removed.
	 */
	[0 ... __NR_ia32_syscall_max] = &compat_ni_syscall,
#include <asm/syscalls_32.h>
};
