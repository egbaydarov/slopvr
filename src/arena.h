/* Arena allocator: one reserved address range per arena (mmap PROT_NONE), committed page by page as it is used, bump
 * allocation, no individual frees.  Temporary scopes rewind the arena; thread local scratch arenas serve short lived
 * work inside a function.  Everything in slopvr that needs heap memory goes through this. */
#ifndef SLOP_ARENA_H
#define SLOP_ARENA_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/mman.h>

struct arena {
	uint8_t *base;    /* start of the reserved range */
	size_t reserved;  /* bytes reserved */
	size_t committed; /* bytes readable/writable so far, page multiple */
	size_t pos;       /* first free byte */
	size_t block;     /* commit granularity */
};

struct arena_temp { struct arena *a; size_t pos; };

#define ARENA_ALIGN 16
#define ARENA_PAGE 4096u
#define arena_push_array(a, type, n) ((type *)arena_push((a), sizeof(type) * (n)))

static inline size_t arena_round(size_t n, size_t to) { return (n + to - 1) & ~(to - 1); }

/* Reserve `reserve` bytes of address space (nothing is committed yet); `block` is the commit step. NULL on failure. */
static struct arena *arena_create(size_t reserve, size_t block)
{
	reserve = arena_round(reserve, ARENA_PAGE);
	block = arena_round(block ? block : 64 * 1024, ARENA_PAGE);
	void *p = mmap(NULL, reserve, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
	if (p == MAP_FAILED) return NULL;
	size_t first = arena_round(sizeof(struct arena), ARENA_PAGE);
	if (first > block) block = first;
	if (mprotect(p, block, PROT_READ | PROT_WRITE)) { munmap(p, reserve); return NULL; }
	struct arena *a = p; /* the header lives at the start of its own range */
	a->base = p; a->reserved = reserve; a->committed = block; a->block = block;
	a->pos = arena_round(sizeof *a, ARENA_ALIGN);
	return a;
}

static void arena_destroy(struct arena *a) { if (a) munmap(a->base, a->reserved); }

/* Uninitialised memory, 16-byte aligned. NULL when the reserved range is exhausted. */
static void *arena_push(struct arena *a, size_t n)
{
	size_t start = arena_round(a->pos, ARENA_ALIGN), end = start + n;
	if (end > a->reserved) return NULL;
	if (end > a->committed) {
		size_t want = arena_round(end, a->block); if (want > a->reserved) want = a->reserved;
		if (mprotect(a->base + a->committed, want - a->committed, PROT_READ | PROT_WRITE)) return NULL;
		a->committed = want;
	}
	a->pos = end;
	return a->base + start;
}

static void *arena_push_zero(struct arena *a, size_t n)
{
	void *p = arena_push(a, n);
	if (p) memset(p, 0, n); /* fresh pages are zero already, rewound ranges are not */
	return p;
}

/* Rewind to `pos`; the memory stays committed for reuse. */
static void arena_pop_to(struct arena *a, size_t pos) { if (pos < a->pos) a->pos = pos; }
static void arena_reset(struct arena *a) { arena_pop_to(a, arena_round(sizeof *a, ARENA_ALIGN)); }

static struct arena_temp arena_temp_begin(struct arena *a) { return (struct arena_temp){ a, a->pos }; }
static void arena_temp_end(struct arena_temp t) { arena_pop_to(t.a, t.pos); }

/* Scratch: a per-thread arena for temporaries.  arena_scratch_begin() opens a temporary scope on it, arena_scratch_end()
 * rewinds; scopes nest.  Do not return scratch memory to a caller that may itself be using the scratch arena. */
#define ARENA_SCRATCH_RESERVE (64u << 20)
static __thread struct arena *arena_scratch_tls;
static struct arena_temp arena_scratch_begin(void)
{
	if (!arena_scratch_tls) arena_scratch_tls = arena_create(ARENA_SCRATCH_RESERVE, 256 * 1024);
	if (!arena_scratch_tls) return (struct arena_temp){ NULL, 0 };
	return arena_temp_begin(arena_scratch_tls);
}
static void arena_scratch_end(struct arena_temp t) { if (t.a) arena_temp_end(t); }

#endif
