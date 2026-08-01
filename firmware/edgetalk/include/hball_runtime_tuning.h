#ifndef HBALL_RUNTIME_TUNING_H
#define HBALL_RUNTIME_TUNING_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool hball_runtime_tuning_set(const char *name, float value);

#ifdef __cplusplus
}
#endif

#endif
