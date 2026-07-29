#ifndef HBALL_DUALCORE_PLATFORM_H
#define HBALL_DUALCORE_PLATFORM_H

#include "hball_dualcore_ipc.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_IPC_SHARED_SRAM_BASE UINT32_C(0x261c0000)
#define HBALL_IPC_SHARED_SRAM_SIZE UINT32_C(0x00040000)
#define HBALL_IPC_SHARED_SECTION_NAME ".hball_ipc_shared"

hball_ipc_shared_region_t *hball_ipc_platform_region(void);
const hball_ipc_cache_ops_t *hball_ipc_platform_cache_ops(void);

#ifdef __cplusplus
}
#endif

#endif
