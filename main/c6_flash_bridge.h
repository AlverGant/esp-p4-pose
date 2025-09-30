#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Returns true if bridge took over (app should return)
bool c6_flash_bridge_try_start(void);

#ifdef __cplusplus
}
#endif

