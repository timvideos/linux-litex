/*
 * OpenRISC Linux
 *
 * Linux architectural port borrowing liberally from similar works of
 * others.  All original copyrights apply as per the original source
 * declaration.
 *
 * OpenRISC implementation:
 * Copyright (C) Jan Henrik Weinstock <jan.weinstock@rwth-aachen.de>
 * et al.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ASM_CACHEFLUSH_H
#define __ASM_CACHEFLUSH_H

#include <linux/mm.h>

/* 
 * Helper function for flushing or invalidating entire pages from data
 * and instruction caches. SMP needs a little extra work, since we need
 * to flush the pages on all cpus.
 */
extern void local_dcache_page_flush(struct page *page);
extern void local_icache_page_inv(struct page *page);

/*
 * For uniprocessor configurations, flush and invalidate functions just
 * map to the local_XXX version. For SMP, we need to flush the caches on
 * every cpu, which is done using the smp_XXX functions.
 */
#ifndef CONFIG_SMP
#define dcache_page_flush(page)      local_dcache_page_flush(page)
#define icache_page_inv(page)        local_icache_page_inv(page)
#else  /* !CONFIG_SMP */
#define dcache_page_flush(page)      smp_dcache_page_flush(page)
#define icache_page_inv(page)        smp_icache_page_inv(page)
extern void smp_dcache_page_flush(struct page *page);
extern void smp_icache_page_inv(struct page *page);
#endif /* CONFIG_SMP */

/*
 * Pages with this bit set need not be flushed/invalidated, since
 * they have not changed since last flush. New pages start with
 * PG_arch_1 not set and are therefore dirty by default.
 */
#define PG_dc_clean                  PG_arch_1

#define ARCH_IMPLEMENTS_FLUSH_DCACHE_PAGE 1
static inline void flush_dcache_page(struct page *page)
{
        clear_bit(PG_dc_clean, &page->flags);
}

/*
 * Other interfaces are not required since we do not have virtually
 * indexed or tagged caches. So we can use the default here.
 */
#define flush_cache_all()			do { } while (0)
#define flush_cache_mm(mm)                      do { } while (0)
#define flush_cache_dup_mm(mm)			do { } while (0)
#define flush_cache_range(vma, start, end)	do { } while (0)
#define flush_cache_page(vma, vmaddr, pfn)	do { } while (0)
#define flush_dcache_mmap_lock(mapping)		do { } while (0)
#define flush_dcache_mmap_unlock(mapping)	do { } while (0)
#define flush_icache_range(start, end)		do { } while (0)
#define flush_icache_page(vma,pg)		do { } while (0)
#define flush_icache_user_range(vma,pg,adr,len)	do { } while (0)
#define flush_cache_vmap(start, end)		do { } while (0)
#define flush_cache_vunmap(start, end)		do { } while (0)

#define copy_to_user_page(vma, page, vaddr, dst, src, len) \
	do { \
		memcpy(dst, src, len); \
		flush_icache_user_range(vma, page, vaddr, len); \
	} while (0)
#define copy_from_user_page(vma, page, vaddr, dst, src, len) \
	memcpy(dst, src, len)

#endif /* __ASM_CACHEFLUSH_H */
