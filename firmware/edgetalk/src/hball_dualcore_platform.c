#include "hball_dualcore_platform.h"

#include "cy_device_headers.h"

#include <limits.h>
#include <stdint.h>

#if !defined(__GNUC__)
#error "H-ball IPC shared-section binding currently requires the GCC toolchain"
#endif

__attribute__((section(HBALL_IPC_SHARED_SECTION_NAME)))
__attribute__((aligned(HBALL_IPC_CACHE_LINE_SIZE)))
__attribute__((used))
static hball_ipc_shared_region_t g_hball_ipc_shared_region;

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
static void hball_ipc_aligned_range(
    void *address, size_t length, uintptr_t *start, int32_t *span
)
{
    const uintptr_t mask = (uintptr_t)HBALL_IPC_CACHE_LINE_SIZE - 1U;
    const uintptr_t address_value = (uintptr_t)address;
    const uintptr_t aligned_start = address_value & ~mask;
    uintptr_t aligned_end;

    if ((length == 0U) || (address_value > UINTPTR_MAX - length)
        || ((address_value + length) > UINTPTR_MAX - mask))
    {
        *start = aligned_start;
        *span = 0;
        return;
    }
    aligned_end = (address_value + length + mask) & ~mask;
    *start = aligned_start;
    *span = (aligned_end - aligned_start) > (uintptr_t)INT32_MAX
        ? 0
        : (int32_t)(aligned_end - aligned_start);
}
#endif

static void hball_ipc_platform_clean(
    void *address, size_t length, void *context
)
{
    (void)context;
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t start;
    int32_t span;

    hball_ipc_aligned_range(address, length, &start, &span);
    if (span > 0)
    {
        SCB_CleanDCache_by_Addr((volatile void *)start, span);
    }
#else
    (void)address;
    (void)length;
#endif
}

static void hball_ipc_platform_invalidate(
    void *address, size_t length, void *context
)
{
    (void)context;
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t start;
    int32_t span;

    hball_ipc_aligned_range(address, length, &start, &span);
    if (span > 0)
    {
        SCB_InvalidateDCache_by_Addr((volatile void *)start, span);
    }
#else
    (void)address;
    (void)length;
#endif
}

static void hball_ipc_platform_barrier(void *context)
{
    (void)context;
    __DMB();
    __DSB();
}

static const hball_ipc_cache_ops_t g_hball_ipc_cache_ops = {
    .clean = hball_ipc_platform_clean,
    .invalidate = hball_ipc_platform_invalidate,
    .barrier = hball_ipc_platform_barrier,
    .context = NULL,
};

hball_ipc_shared_region_t *hball_ipc_platform_region(void)
{
    return &g_hball_ipc_shared_region;
}

const hball_ipc_cache_ops_t *hball_ipc_platform_cache_ops(void)
{
    return &g_hball_ipc_cache_ops;
}
