#ifndef DYNAMIC_MODELS_UTILITIES_MEMORY_ARENA_H
#define DYNAMIC_MODELS_UTILITIES_MEMORY_ARENA_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dm_arena_s {
    uint8_t* base;
    size_t capacity;
    size_t offset;
} dm_arena_t;

static inline void dm_arena_init(dm_arena_t* arena, void* memory, size_t bytes) {
    arena->base = (uint8_t*)memory;
    arena->capacity = bytes;
    arena->offset = 0U;
}

static inline size_t dm_arena_align(size_t offset, size_t alignment) {
    return (offset + alignment - 1U) & ~(alignment - 1U);
}

static inline void* dm_arena_alloc(dm_arena_t* arena, size_t size, size_t alignment) {
    size_t aligned = dm_arena_align(arena->offset, alignment);
    size_t next_offset = aligned + size;
    if (next_offset > arena->capacity) {
        return NULL;
    }
    arena->offset = next_offset;
    return arena->base + aligned;
}

#ifdef __cplusplus
}
#endif

#endif /* DYNAMIC_MODELS_UTILITIES_MEMORY_ARENA_H */
