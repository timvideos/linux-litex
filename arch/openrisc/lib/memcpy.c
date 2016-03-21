/*
 * arch/openrisc/lib/memcpy.c
 *
 * Optimized memory copy routines for openrisc.  These are mostly copied
 * from ohter sources but slightly entended based on ideas discuassed in
 * #openrisc.
 *
 * The word non aligned is based on microblaze found in:
 *  arch/microblaze/lib/memcpy.c
 * but this is extended to have loop unrolls. This only supports
 * big endian at the moment.
 *
 * The byte unroll implementation is a copy of that found in:
 *  arm/boot/compressed/string.c
 *
 * The word unroll implementation is an extention to the byte
 * unrolled implementation, but using word copies (if things are
 * properly aligned)
 */

#ifndef _MC_TEST
#include <linux/string.h>
#endif

#if defined(CONFIG_OPT_LIB_WORD_NONALIGNED)
/*
 * Make below loops a bit more manageable
 *
 *
 */
#define __OFFSET_MEMCPY(n) 	value = *src_w++;					\
				*dest_w++ = buf_hold | value >> ( ( 4 - n ) * 8 );	\
				buf_hold = value << ( n * 8 )

void *memcpy(void *dest, const void *src, __kernel_size_t n)
{
	const char *src_b = src;
	char *dest_b = dest;
	int i;

	/* The following code tries to optimize the copy by using unsigned
	 * alignment. This will work fine if both source and destination are
	 * aligned on the same boundary. However, if they are aligned on
	 * different boundaries shifts will be necessary.
	 */
	const uint32_t *src_w;
	uint32_t *dest_w;

	if (likely(n >= 4)) {
		unsigned  value, buf_hold;

		/* Align the destination to a word boundary. */
		/* This is done in an endian independent manner. */
		switch ((unsigned long)dest_b & 3) {
		case 1:
			*dest_b++ = *src_b++;
			--n;
		case 2:
			*dest_b++ = *src_b++;
			--n;
		case 3:
			*dest_b++ = *src_b++;
			--n;
		}

		dest_w = (void *)dest_b;

		/* Choose a copy scheme based on the source, this is done big endian */
		/* alignment relative to destination. */
		switch ((unsigned long)src_b & 3) {
		case 0x0:	/* Both byte offsets are aligned */
			src_w  = (const uint32_t *)src_b;

			/* Copy 32 bytes per loop */
			for (i = n >> 5; i > 0; i--) {
				*dest_w++ = *src_w++;
				*dest_w++ = *src_w++;
				*dest_w++ = *src_w++;
				*dest_w++ = *src_w++;
				*dest_w++ = *src_w++;
				*dest_w++ = *src_w++;
				*dest_w++ = *src_w++;
				*dest_w++ = *src_w++;
			}

			if (n & 1 << 4) {
				*dest_w++ = *src_w++;
				*dest_w++ = *src_w++;
				*dest_w++ = *src_w++;
				*dest_w++ = *src_w++;
			}

			if (n & 1 << 3) {
				*dest_w++ = *src_w++;
				*dest_w++ = *src_w++;
			}

			if (n & 1 << 2)
				*dest_w++ = *src_w++;

			src_b  = (const char *)src_w;
			break;

		case 0x1:	/* Unaligned - Off by 1 */
			/* Word align the source */
			src_w = (const void *) ((unsigned)src_b & ~3);
			/* Load the holding buffer */
			buf_hold = *src_w++ << 8;

			for (i = n >> 5; i > 0; i--) {
				__OFFSET_MEMCPY(1);
				__OFFSET_MEMCPY(1);
				__OFFSET_MEMCPY(1);
				__OFFSET_MEMCPY(1);
				__OFFSET_MEMCPY(1);
				__OFFSET_MEMCPY(1);
				__OFFSET_MEMCPY(1);
				__OFFSET_MEMCPY(1);
			}

			if (n & 1 << 4) {
				__OFFSET_MEMCPY(1);
				__OFFSET_MEMCPY(1);
				__OFFSET_MEMCPY(1);
				__OFFSET_MEMCPY(1);
			}

			if (n & 1 << 3) {
				__OFFSET_MEMCPY(1);
				__OFFSET_MEMCPY(1);
			}

			if (n & 1 << 2) {
				__OFFSET_MEMCPY(1);
			}

			/* Realign the source */
			src_b = (const void *)src_w;
			src_b -= 3;
			break;
		case 0x2:	/* Unaligned - Off by 2 */
			/* Word align the source */
			src_w = (const void *) ((unsigned)src_b & ~3);
			/* Load the holding buffer */
			buf_hold = *src_w++ << 16;

			for (i = n >> 5; i > 0; i--) {
				__OFFSET_MEMCPY(2);
				__OFFSET_MEMCPY(2);
				__OFFSET_MEMCPY(2);
				__OFFSET_MEMCPY(2);
				__OFFSET_MEMCPY(2);
				__OFFSET_MEMCPY(2);
				__OFFSET_MEMCPY(2);
				__OFFSET_MEMCPY(2);
			}

			if (n & 1 << 4) {
				__OFFSET_MEMCPY(2);
				__OFFSET_MEMCPY(2);
				__OFFSET_MEMCPY(2);
				__OFFSET_MEMCPY(2);
			}

			if (n & 1 << 3) {
				__OFFSET_MEMCPY(2);
				__OFFSET_MEMCPY(2);
			}

			if (n & 1 << 2) {
				__OFFSET_MEMCPY(2);
			}

			/* Realign the source */
			src_b = (const void *)src_w;
			src_b -= 2;
			break;
		case 0x3:	/* Unaligned - Off by 3 */
			/* Word align the source */
			src_w = (const void *) ((unsigned)src_b & ~3);
			/* Load the holding buffer */
			buf_hold = *src_w++ << 24;

			for (i = n >> 5; i > 0; i--) {
				__OFFSET_MEMCPY(3);
				__OFFSET_MEMCPY(3);
				__OFFSET_MEMCPY(3);
				__OFFSET_MEMCPY(3);
				__OFFSET_MEMCPY(3);
				__OFFSET_MEMCPY(3);
				__OFFSET_MEMCPY(3);
				__OFFSET_MEMCPY(3);
			}

			if (n & 1 << 4) {
				__OFFSET_MEMCPY(3);
				__OFFSET_MEMCPY(3);
				__OFFSET_MEMCPY(3);
				__OFFSET_MEMCPY(3);
			}

			if (n & 1 << 3) {
				__OFFSET_MEMCPY(3);
				__OFFSET_MEMCPY(3);
			}

			if (n & 1 << 2) {
				__OFFSET_MEMCPY(3);
			}

			/* Realign the source */
			src_b = (const void *)src_w;
			src_b -= 1;
			break;
		}
		dest_b = (void *)dest_w;
	}

	/* Finish off any remaining bytes */
	/* simple fast copy, ... unless a cache boundary is crossed */
       	if (n & 1 << 1) {
		*dest_b++ = *src_b++;
		*dest_b++ = *src_b++;
	}

	if (n & 1)
		*dest_b++ = *src_b++;


	return dest;
}
#elif defined(CONFIG_OPT_LIB_WORD)
void * memcpy(void *dest, __const void *src, __kernel_size_t n)
{
	unsigned char * d = (unsigned char *) dest, * s = (unsigned char *) src;
	uint32_t * dest_w = (uint32_t *) dest, * src_w = (uint32_t *) src;

	/* If both source and dest are word aligned copy words */
	if (!((unsigned)dest_w & 3) && !((unsigned)src_w & 3)) {
		for (; n >= 4; n -= 4)
			*dest_w++ = *src_w++;
	}

	d = (unsigned char *) dest_w;
	s = (unsigned char *) src_w;

	/* For remaining or if not aligned, copy bytes */
	for (; n >= 1; n -= 1)
		*d++ = *s++;

	return dest;

}
#elif defined(CONFIG_OPT_LIB_WORD_UNROLL)
void * memcpy(void *dest, __const void *src, __kernel_size_t n)
{
	int i = 0;
	unsigned char * d, * s;
	uint32_t * dest_w = (uint32_t *) dest, * src_w = (uint32_t *) src;

	/* If both source and dest are word aligned copy words */
	if (!((unsigned)dest_w & 3) && !((unsigned)src_w & 3)) {
		/* Copy 32 bytes per loop */
		for (i = n >> 5; i > 0; i--) {
			*dest_w++ = *src_w++;
			*dest_w++ = *src_w++;
			*dest_w++ = *src_w++;
			*dest_w++ = *src_w++;
			*dest_w++ = *src_w++;
			*dest_w++ = *src_w++;
			*dest_w++ = *src_w++;
			*dest_w++ = *src_w++;
		}

		if (n & 1 << 4) {
			*dest_w++ = *src_w++;
			*dest_w++ = *src_w++;
			*dest_w++ = *src_w++;
			*dest_w++ = *src_w++;
		}

		if (n & 1 << 3) {
			*dest_w++ = *src_w++;
			*dest_w++ = *src_w++;
		}

		if (n & 1 << 2)
			*dest_w++ = *src_w++;

		d = (unsigned char *) dest_w;
		s = (unsigned char *) src_w;

	} else {
		d = (unsigned char *) dest_w;
		s = (unsigned char *) src_w;

		for (i = n >> 3; i > 0; i--) {
			*d++ = *s++;
			*d++ = *s++;
			*d++ = *s++;
			*d++ = *s++;
			*d++ = *s++;
			*d++ = *s++;
			*d++ = *s++;
			*d++ = *s++;
		 }

		 if (n & 1 << 2) {
			*d++ = *s++;
			*d++ = *s++;
			*d++ = *s++;
			*d++ = *s++;
		 }
	}

       	if (n & 1 << 1) {
		*d++ = *s++;
		*d++ = *s++;
	}

	if (n & 1)
		*d++ = *s++;

	return dest;
}

#elif defined(CONFIG_OPT_LIB_BYTE_UNROLL)
void * memcpy(void *dest, __const void *src, __kernel_size_t n)
{
	int i = 0;
	unsigned char * d = (unsigned char *) dest, * s = (unsigned char *) src;

	/* For remaining or if not aligned, still unroll loops */
	for (i = n >> 3; i > 0; i--) {
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
	}

	if (n & 1 << 2) {
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
	}

	if (n & 1 << 1) {
		*d++ = *s++;
		*d++ = *s++;
	}

	if (n & 1)
		*d++ = *s++;

	return dest;
}
#else /* CONFIG_OPT_LIB_BYTE fallback */
void * memcpy(void *dest, __const void *src, __kernel_size_t n)
{
	unsigned char * d = (unsigned char *) dest, * s = (unsigned char *) src;

	/* For remaining or if not aligned, still unroll loops */
	for (; n > 0; n--)
		*d++ = *s++;

	return dest;
}
#endif
