from __future__ import annotations

from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[3]
FREERTOS = ROOT / "firmware" / "edgetalk" / "freertos"
TASK = FREERTOS / "hball_m55_shadow_task.c"
MAKE_FRAGMENT = FREERTOS / "Makefile.hball.mk"
CONFIG = FREERTOS / "FreeRTOSConfig.h"
DEPENDENCY = FREERTOS / "deps" / "freertos.mtb"
CMSIS_DEPENDENCY = FREERTOS / "deps" / "cmsis.mtb"
ASYNC_TRANSFER_DEPENDENCY = FREERTOS / "deps" / "async-transfer.mtb"


def test_m55_freertos_task_runs_200hz_shadow_and_10hz_ui_without_actuators():
    source = TASK.read_text(encoding="utf-8")

    assert '#include "FreeRTOS.h"' in source
    assert '#include "task.h"' in source
    assert "#define HBALL_M55_PERIOD_MS 5U" in source
    assert "#define HBALL_M55_UI_PERIOD_MS 100U" in source
    assert "xTaskCreate(" in source
    assert "vTaskDelayUntil(" in source
    assert "hball_m55_read_sensor_snapshot(" in source
    assert "hball_control_pipeline_step(" in source
    assert "hball_m55_publish_control_shadow(" in source
    assert "HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY" in source
    assert "HBALL_IPC_CONTROL_FLAG_SAFETY_ELIGIBLE" in source
    assert "hball_m55_platform_ui_10hz(" in source
    assert "HBALL_UI_VALID_MOTOR_PARAMETERS" in source
    assert "motor_filtered_iq_a" in source
    assert "motor_vbus_v" in source
    assert "motor_temperature_c" in source
    assert "ACTUATOR_TX=0" in source

    lowered = source.lower()
    for forbidden in [
        "rtthread",
        "rt_thread",
        "finsh",
        "ifx_can",
        "rt_device_write",
        "cy_canfd_updat",
        "motor_command",
    ]:
        assert forbidden not in lowered


def test_m55_modustoolbox_fragment_pins_real_cm55_freertos_port():
    make = MAKE_FRAGMENT.read_text(encoding="utf-8")
    config = CONFIG.read_text(encoding="utf-8")
    dependency = DEPENDENCY.read_text(encoding="utf-8").strip()
    cmsis_dependency = CMSIS_DEPENDENCY.read_text(encoding="utf-8").strip()
    async_transfer_dependency = ASYNC_TRANSFER_DEPENDENCY.read_text(
        encoding="utf-8"
    ).strip()

    assert "CORE=CM55" in make
    assert "CORE_NAME=CM55_0" in make
    assert "COMPONENTS+=FREERTOS" in make.replace(" ", "")
    assert "VFP_SELECT=hardfp" in make
    assert "VFP_SELECT_PRECISION=doublefp" in make
    assert "MVE_SELECT=NO_MVE" in make
    assert "hball_m55_shadow_task.c" in make
    assert "hball_m55_ipc.c" in make
    assert "hball_bench_app.c" not in make
    assert "hball_can.c" not in make
    assert "LDLIBS+=-lm" in make.replace(" ", "")

    assert "configTICK_RATE_HZ" in config and "1000" in config
    assert "INCLUDE_vTaskDelayUntil" in config
    assert "configENABLE_FPU" in config
    assert "configENABLE_MVE" in config
    assert "configENABLE_TRUSTZONE" in config
    assert "configENABLE_MPU" in config
    assert "#define INCLUDE_xTaskGetCurrentTaskHandle 1" in config

    assert dependency == (
        "https://github.com/Infineon/freertos"
        "#8a19c8db81becf1e981a5f94630952160fddf8c5"
        "#$$ASSET_REPO$$/freertos/release-v10.6.202"
    )
    assert cmsis_dependency == (
        "https://github.com/Infineon/cmsis"
        "#release-v6.1.0"
        "#$$ASSET_REPO$$/cmsis/release-v6.1.0"
    )
    assert async_transfer_dependency == (
        "https://github.com/Infineon/async-transfer"
        "#release-v1.1.1"
        "#$$ASSET_REPO$$/async-transfer/release-v1.1.1"
    )


def test_m55_freertos_adapter_compiles_against_task_api(tmp_path: Path):
    gcc = shutil.which("gcc")
    assert gcc is not None, "host GCC is required for adapter compile checks"

    (tmp_path / "FreeRTOS.h").write_text(
        """
#ifndef FREERTOS_H
#define FREERTOS_H
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
#define pdPASS 1
#define configMAX_PRIORITIES 7U
#define configTICK_RATE_HZ 1000U
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
""".strip(),
        encoding="utf-8",
    )
    (tmp_path / "task.h").write_text(
        """
#ifndef TASK_H
#define TASK_H
#include "FreeRTOS.h"
typedef void (*TaskFunction_t)(void *);
BaseType_t xTaskCreate(TaskFunction_t, const char *, uint32_t, void *,
                       uint32_t, TaskHandle_t *);
TickType_t xTaskGetTickCount(void);
BaseType_t xTaskDelayUntil(TickType_t *, TickType_t);
void vTaskDelayUntil(TickType_t *, TickType_t);
#endif
""".strip(),
        encoding="utf-8",
    )

    subprocess.run(
        [
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
            "-c",
            str(TASK),
            "-o",
            str(tmp_path / "hball_m55_shadow_task.o"),
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
