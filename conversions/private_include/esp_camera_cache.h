#pragma once

#include <stddef.h>

#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#endif

static inline size_t esp_camera_dcache_line_size(void) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
  size_t line =
      cache_hal_get_cache_line_size(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_DATA);
  return line != 0 ? line : 32;
#else
  return 32;
#endif
}
