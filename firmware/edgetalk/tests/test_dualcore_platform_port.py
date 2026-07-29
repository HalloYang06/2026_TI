from __future__ import annotations

from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[3]


def test_dualcore_platform_binds_shared_section_and_cache_hooks(tmp_path: Path):
    gcc = shutil.which("gcc")
    assert gcc is not None, "host GCC is required for the EdgeTalk C-port tests"

    fake_cmsis = tmp_path / "cy_device_headers.h"
    fake_cmsis.write_text(
        """
#ifndef CY_DEVICE_HEADERS_H
#define CY_DEVICE_HEADERS_H
#include <stdint.h>
#define __DCACHE_PRESENT 1U
#define __SCB_DCACHE_LINE_SIZE 32U
void hball_test_clean(volatile void *address, int32_t length);
void hball_test_invalidate(volatile void *address, int32_t length);
void hball_test_dmb(void);
void hball_test_dsb(void);
#define SCB_CleanDCache_by_Addr(address, length) hball_test_clean(address, length)
#define SCB_InvalidateDCache_by_Addr(address, length) hball_test_invalidate(address, length)
#define __DMB() hball_test_dmb()
#define __DSB() hball_test_dsb()
#endif
""".strip(),
        encoding="utf-8",
    )
    harness = tmp_path / "platform_test.c"
    harness.write_text(
        """
#include "hball_dualcore_platform.h"

#include <assert.h>
#include <stdint.h>

static uintptr_t clean_address;
static int32_t clean_length;
static uintptr_t invalidate_address;
static int32_t invalidate_length;
static unsigned dmb_total;
static unsigned dsb_total;

void hball_test_clean(volatile void *address, int32_t length)
{
    clean_address = (uintptr_t)address;
    clean_length = length;
}

void hball_test_invalidate(volatile void *address, int32_t length)
{
    invalidate_address = (uintptr_t)address;
    invalidate_length = length;
}

void hball_test_dmb(void) { dmb_total++; }
void hball_test_dsb(void) { dsb_total++; }

int main(void)
{
    hball_ipc_shared_region_t *region = hball_ipc_platform_region();
    const hball_ipc_cache_ops_t *ops = hball_ipc_platform_cache_ops();
    uintptr_t base = (uintptr_t)region;

    assert(region != 0);
    assert((base % HBALL_IPC_CACHE_LINE_SIZE) == 0U);
    assert(ops != 0);
    assert(ops->clean != 0);
    assert(ops->invalidate != 0);
    assert(ops->barrier != 0);

    ops->clean((void *)(base + 1U), 33U, ops->context);
    assert(clean_address == base);
    assert(clean_length == 64);
    ops->invalidate((void *)(base + 31U), 2U, ops->context);
    assert(invalidate_address == base);
    assert(invalidate_length == 64);
    ops->barrier(ops->context);
    assert(dmb_total == 1U);
    assert(dsb_total == 1U);
    return 0;
}
""".strip(),
        encoding="utf-8",
    )

    executable = tmp_path / "hball_dualcore_platform_test.exe"
    command = [
        gcc,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        "-I",
        str(tmp_path),
        "-I",
        str(ROOT / "firmware" / "edgetalk" / "include"),
        str(
            ROOT
            / "firmware"
            / "edgetalk"
            / "src"
            / "hball_dualcore_platform.c"
        ),
        str(harness),
        "-o",
        str(executable),
    ]
    subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    subprocess.run(
        [str(executable)], cwd=ROOT, check=True, capture_output=True, text=True
    )


def test_both_core_build_lists_include_platform_transport():
    m33 = (ROOT / "firmware" / "edgetalk" / "SConscript").read_text(
        encoding="utf-8"
    )
    m55 = (ROOT / "firmware" / "edgetalk" / "SConscript.m55").read_text(
        encoding="utf-8"
    )

    for build_script in (m33, m55):
        assert "hball_dualcore_ipc.c" in build_script
        assert "hball_dualcore_platform.c" in build_script


def test_linker_fragment_reserves_exactly_one_region_at_shared_sram_base():
    fragment = (
        ROOT
        / "firmware"
        / "edgetalk"
        / "linker"
        / "hball_dualcore_section.ld.inc"
    ).read_text(encoding="utf-8")

    assert ".hball_ipc_shared" in fragment
    assert "KEEP(*(.hball_ipc_shared))" in fragment
    assert "> m33_m55_shared" in fragment
    assert "ORIGIN(m33_m55_shared)" in fragment
    assert "SIZEOF(.hball_ipc_shared) == 0x100" in fragment
