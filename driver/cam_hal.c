// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include <stdalign.h>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ll_cam.h"
#include "cam_hal.h"
#include "esp_camera_cache.h"

#if (ESP_IDF_VERSION_MAJOR == 3) && (ESP_IDF_VERSION_MINOR == 3)
#include "rom/ets_sys.h"
#else
#include "esp_timer.h"
#include "esp_cache.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include "esp_idf_version.h"
#ifndef ESP_CACHE_MSYNC_FLAG_DIR_M2C
#define ESP_CACHE_MSYNC_FLAG_DIR_M2C 0
#endif
#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/ets_sys.h"  // will be removed in idf v5.0
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/ets_sys.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/ets_sys.h"
#endif
#endif // ESP_IDF_VERSION_MAJOR

#if CONFIG_LOG_DEFAULT_LEVEL_NONE
#define ESP_CAMERA_ETS_PRINTF(f, ...)
#else
#define ESP_CAMERA_ETS_PRINTF(f, ...) ets_printf(f, ##__VA_ARGS__)
#endif

#if CONFIG_CAMERA_TASK_STACK_SIZE
#define CAM_TASK_STACK             CONFIG_CAMERA_TASK_STACK_SIZE
#else
#define CAM_TASK_STACK             (4*1024)
#endif

static const char *TAG = "cam_hal";
static cam_obj_t *cam_obj = NULL;
#if defined(CONFIG_CAMERA_PSRAM_DMA)
#define CAMERA_PSRAM_DMA_ENABLED CONFIG_CAMERA_PSRAM_DMA
#else
#define CAMERA_PSRAM_DMA_ENABLED 0
#endif

static volatile bool g_dma_mode = CAMERA_PSRAM_DMA_ENABLED;
static portMUX_TYPE g_dma_mode_lock = portMUX_INITIALIZER_UNLOCKED;

/* At top of cam_hal.c – one switch for noisy ISR prints */
#ifndef CAM_LOG_SPAM_EVERY_FRAME
#define CAM_LOG_SPAM_EVERY_FRAME 0   /* set to 1 to restore old behaviour */
#endif

/* Number of bytes copied to SRAM for SOI validation when capturing
 * directly to PSRAM. Tunable to probe more of the frame start if needed. */
#ifndef CAM_SOI_PROBE_BYTES
#define CAM_SOI_PROBE_BYTES 32
#endif
/*
 * PSRAM DMA may bypass the CPU cache. Always call esp_cache_msync() on
 * PSRAM regions that the CPU will read so cached reads see the data written
 * by DMA.
 *
 * Invalidate CPU data cache lines that cover a region in PSRAM which
 * has just been written by DMA. This guarantees subsequent CPU reads
 * fetch the fresh data from PSRAM rather than stale cache contents.
 * Both address and length are aligned to the data cache line size.
 */
static inline void cam_drop_psram_cache(void *addr, size_t len)
{
    size_t line = esp_camera_dcache_line_size();
    uintptr_t start = (uintptr_t)addr & ~(line - 1);
    size_t sync_len = (len + ((uintptr_t)addr - start) + line - 1) & ~(line - 1);
    esp_err_t err = esp_cache_msync((void *)start, sync_len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    if (err != ESP_OK) {
        ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: cache msync failed addr=%p len=%u aligned=%p aligned_len=%u err=%d\r\n"),
                              addr, (unsigned)len, (void *)start, (unsigned)sync_len, (int)err);
    }
}

/* Throttle repeated warnings printed from tight loops / ISRs.
 *
 * counter – static DRAM/IRAM uint16_t you pass in
 * first   – literal C string shown on first hit and as prefix of summaries
 */
#if CONFIG_LOG_DEFAULT_LEVEL >= 2
#define CAM_WARN_THROTTLE(counter, first)                                  \
    do {                                                                  \
        if (++(counter) == 1) {                                           \
            ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: %s\r\n"), first);        \
        } else if ((counter) % 100 == 0) {                                \
            ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: %s - 100 additional misses\r\n"), first); \
        }                                                                 \
        if ((counter) == 10000) (counter) = 1;                            \
    } while (0)
#else
#define CAM_WARN_THROTTLE(counter, first) do { (void)(counter); } while (0)
#endif

/* JPEG markers (byte-order independent). */
static const uint8_t JPEG_SOI_MARKER[] = {0xFF, 0xD8, 0xFF}; /* SOI = FF D8 FF */
#define JPEG_SOI_MARKER_LEN (3)
static const uint8_t JPEG_EOI_BYTES[] = {0xFF, 0xD9};        /* EOI = FF D9 */
#define JPEG_EOI_MARKER_LEN (2)

static int cam_find_jpeg_soi(const uint8_t *inbuf, uint32_t length)
{
    if (length < JPEG_SOI_MARKER_LEN) {
        return -1;
    }

    for (uint32_t i = 0; i <= length - JPEG_SOI_MARKER_LEN; i++) {
        if (memcmp(&inbuf[i], JPEG_SOI_MARKER, JPEG_SOI_MARKER_LEN) == 0) {
            return i;
        }
    }

    return -1;
}

static int cam_verify_jpeg_soi(const uint8_t *inbuf, uint32_t length)
{
    static uint16_t warn_soi_miss_cnt = 0;

    int soi_off = cam_find_jpeg_soi(inbuf, length);
    if (soi_off >= 0) {
        return soi_off;
    }

    CAM_WARN_THROTTLE(warn_soi_miss_cnt,
                      length < JPEG_SOI_MARKER_LEN
                          ? "NO-SOI - JPEG start marker missing (len < 3b)"
                          : "NO-SOI - JPEG start marker missing");
    return -1;
}

static int cam_verify_jpeg_eoi(const uint8_t *inbuf, uint32_t length, bool search_forward)
{
    if (length < JPEG_EOI_MARKER_LEN) {
        return -1;
    }

    if (search_forward) {
        /* Scan forward to honor the earliest marker in the buffer. This avoids
         * returning an EOI that belongs to a larger previous frame when the tail
         * of that frame still resides in PSRAM. JPEG data is pseudo random, so
         * the first marker byte appears rarely; test four positions per load to
         * reduce memory traffic. */
        const uint8_t *pat = JPEG_EOI_BYTES;
        const uint32_t A = pat[0] * 0x01010101u;
        const uint32_t ONE = 0x01010101u;
        const uint32_t HIGH = 0x80808080u;
        uint32_t i = 0;
        while (i + 4 <= length) {
            uint32_t w;
            memcpy(&w, inbuf + i, 4); /* unaligned load is allowed */
            uint32_t x = w ^ A; /* identify bytes equal to first marker byte */
            uint32_t m = (~x & (x - ONE)) & HIGH; /* mask has high bit set for candidate bytes */
            while (m) { /* handle only candidates to avoid unnecessary memcmp calls */
                unsigned off = __builtin_ctz(m) >> 3;
                uint32_t pos = i + off;
                if (pos + JPEG_EOI_MARKER_LEN <= length &&
                    memcmp(inbuf + pos, pat, JPEG_EOI_MARKER_LEN) == 0) {
                    return pos;
                }
                m &= m - 1; /* clear processed candidate */
            }
            i += 4;
        }
        for (; i + JPEG_EOI_MARKER_LEN <= length; i++) {
            if (memcmp(inbuf + i, pat, JPEG_EOI_MARKER_LEN) == 0) {
                return i;
            }
        }
        return -1;
    }

    const uint8_t *dptr = inbuf + length - JPEG_EOI_MARKER_LEN;
    while (dptr >= inbuf) {
        if (memcmp(dptr, JPEG_EOI_BYTES, JPEG_EOI_MARKER_LEN) == 0) {
            return dptr - inbuf;
        }
        if (dptr == inbuf) {
            break;
        }
        dptr--;
    }
    return -1;
}

static bool cam_get_next_frame(int * frame_pos)
{
    if(!cam_obj->frames[*frame_pos].en){
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            if (cam_obj->frames[x].en) {
                *frame_pos = x;
                return true;
            }
        }
    } else {
        return true;
    }
    return false;
}

static bool cam_start_frame(int * frame_pos)
{
    if (cam_get_next_frame(frame_pos)) {
        if(ll_cam_start(cam_obj, *frame_pos)){
            /* LCD_CAM needs a synthetic VSYNC edge after starting a transaction.
             * For JPEG DMA mode, ll_cam_start() temporarily keeps CAM_VS_EOF_EN
             * disabled so this priming pulse cannot complete the frame. */
            ll_cam_do_vsync(cam_obj);
            if (cam_obj->dma_mode && cam_obj->jpeg_mode) {
                ll_cam_set_vsync_eof(cam_obj, true);
            }
            uint64_t us = (uint64_t)esp_timer_get_time();
            cam_obj->frames[*frame_pos].fb.timestamp.tv_sec = us / 1000000UL;
            cam_obj->frames[*frame_pos].fb.timestamp.tv_usec = us % 1000000UL;
            return true;
        }
    }
    return false;
}

void IRAM_ATTR ll_cam_send_event(cam_obj_t *cam, cam_event_t cam_event, BaseType_t * HPTaskAwoken)
{
    if (xQueueSendFromISR(cam->event_queue, (void *)&cam_event, HPTaskAwoken) != pdTRUE) {
        ll_cam_stop(cam);
        cam->state = CAM_STATE_IDLE;
#if CAM_LOG_SPAM_EVERY_FRAME
        ESP_DRAM_LOGD(TAG, "EV-OVF");
#else
        static uint16_t ovf_cnt = 0;
        switch (cam_event) {
        case CAM_IN_SUC_EOF_EVENT:
            CAM_WARN_THROTTLE(ovf_cnt, "EV-EOF-OVF");
            break;
        case CAM_VSYNC_EVENT:
            CAM_WARN_THROTTLE(ovf_cnt, "EV-VSYNC-OVF");
            break;
        case CAM_DMA_ERROR_EVENT:
            CAM_WARN_THROTTLE(ovf_cnt, "EV-DMA-ERR-OVF");
            break;
        case CAM_DMA_FIFO_FULL_EVENT:
            CAM_WARN_THROTTLE(ovf_cnt, "EV-DMA-FIFO-OVF");
            break;
        case CAM_FRAME_RETURNED_EVENT:
            CAM_WARN_THROTTLE(ovf_cnt, "EV-FB-RETURN-OVF");
            break;
        default:
            CAM_WARN_THROTTLE(ovf_cnt, "EV-UNKNOWN-OVF");
            break;
        }
#endif
    }
}

typedef struct {
    size_t len;
    size_t eof_node_len;
    bool eof;
} cam_dma_frame_len_t;

static cam_dma_frame_len_t cam_dma_received_size(const lldesc_t *dma, uint32_t count)
{
    cam_dma_frame_len_t result = { 0, 0, false };

    for (uint32_t i = 0; i < count; i++) {
        result.len += dma[i].length;
        if (dma[i].eof) {
            result.eof_node_len = dma[i].length;
            result.eof = true;
            break;
        }
    }

    return result;
}

static bool cam_queue_frame(camera_fb_t *frame_buffer_event, int frame_pos)
{
    cam_obj->frames[frame_pos].en = 0;

    if (xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) == pdTRUE) {
        if (cam_obj->dma_mode && cam_obj->jpeg_mode) {
            ESP_LOGW(TAG, "JPEG DMA queue frame=%d len=%u q=%u",
                     frame_pos, (unsigned)frame_buffer_event->len,
                     (unsigned)uxQueueMessagesWaiting(cam_obj->frame_buffer_queue));
        }
        return true;
    }

    camera_fb_t *old_frame = NULL;
    if (xQueueReceive(cam_obj->frame_buffer_queue, &old_frame, 0) != pdTRUE) {
        cam_obj->frames[frame_pos].en = 1;
        ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: FBQ-RCV\r\n"));
        return false;
    }

    if (xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) != pdTRUE) {
        cam_obj->frames[frame_pos].en = 1;
        ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: FBQ-SND\r\n"));
        cam_give(old_frame);
        return false;
    }

    if (cam_obj->dma_mode && cam_obj->jpeg_mode) {
        int old_pos = -1;
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            if (&cam_obj->frames[x].fb == old_frame) {
                old_pos = x;
                break;
            }
        }
        ESP_LOGW(TAG, "JPEG DMA replace old=%d new=%d len=%u q=%u",
                 old_pos, frame_pos, (unsigned)frame_buffer_event->len,
                 (unsigned)uxQueueMessagesWaiting(cam_obj->frame_buffer_queue));
    }
    cam_give(old_frame);
    return true;
}

static bool cam_verify_dma_jpeg_start(camera_fb_t *frame_buffer_event)
{
    size_t probe_len = cam_obj->dma_half_buffer_size;
    if (probe_len > CAM_SOI_PROBE_BYTES) {
        probe_len = CAM_SOI_PROBE_BYTES;
    }

    cam_drop_psram_cache(frame_buffer_event->buf, probe_len);

    uint8_t soi_probe[CAM_SOI_PROBE_BYTES];
    memcpy(soi_probe, frame_buffer_event->buf, probe_len);

    int soi_off = cam_find_jpeg_soi(soi_probe, probe_len);
    if (soi_off == 0) {
        return true;
    }

    if (soi_off > 0) {
        ESP_LOGW(TAG, "JPEG DMA reject: SOI not at offset 0, off=%d probe=%u",
                 soi_off, (unsigned)probe_len);
    } else {
        ESP_LOGW(TAG, "JPEG DMA reject: SOI missing, probe=%u", (unsigned)probe_len);
    }
    return false;
}

static bool cam_verify_dma_jpeg_end(camera_fb_t *frame, size_t eof_node_len)
{
    if (frame->len < JPEG_EOI_MARKER_LEN) {
        return false;
    }

    size_t exact_eoi_pos = frame->len - JPEG_EOI_MARKER_LEN;
    if (memcmp(frame->buf + exact_eoi_pos, JPEG_EOI_BYTES, JPEG_EOI_MARKER_LEN) == 0) {
        return true;
    }

    /* The descriptor EOF length should normally make EOI the final two bytes.
     * If it does not, fall back to the narrowest tail probe that can still
     * catch an EOI split across the EOF descriptor boundary: the bytes written
     * into the EOF descriptor plus one byte from the previous descriptor.
     * Never read beyond the GDMA-reported frame length. */
    size_t probe_len = eof_node_len + (JPEG_EOI_MARKER_LEN - 1);
    if (probe_len > frame->len) {
        probe_len = frame->len;
    }

    if (probe_len < JPEG_EOI_MARKER_LEN) {
        return false;
    }

    uint8_t *probe_start = frame->buf + frame->len - probe_len;
    int off = cam_verify_jpeg_eoi(probe_start, probe_len, true);
    if (off < 0) {
        return false;
    }

    size_t recovered_len = frame->len - probe_len + off + JPEG_EOI_MARKER_LEN;
    ESP_LOGW(TAG, "JPEG DMA EOI not at reported tail: len=%u recovered=%u eof_node=%u",
             (unsigned)frame->len, (unsigned)recovered_len, (unsigned)eof_node_len);
    frame->len = recovered_len;
    return true;
}

static bool cam_finish_dma_jpeg_frame(int frame_pos)
{
    camera_fb_t *frame = &cam_obj->frames[frame_pos].fb;
    cam_dma_frame_len_t dma_len = cam_dma_received_size(cam_obj->frames[frame_pos].dma,
                                                        cam_obj->dma_node_cnt);

    ll_cam_stop(cam_obj);

    if (!dma_len.eof) {
        ESP_LOGW(TAG, "JPEG DMA reject: missing EOF descriptor");
        cam_obj->frames[frame_pos].en = 1;
        return false;
    }

    if (dma_len.len == 0 || dma_len.len > cam_obj->fb_size) {
        ESP_LOGW(TAG, "JPEG DMA reject: invalid length len=%u fb=%u",
                 (unsigned)dma_len.len, (unsigned)cam_obj->fb_size);
        cam_obj->frames[frame_pos].en = 1;
        return false;
    }

    frame->len = dma_len.len;
    cam_drop_psram_cache(frame->buf, frame->len);

    if (!cam_verify_dma_jpeg_start(frame)) {
        cam_obj->frames[frame_pos].en = 1;
        return false;
    }

    if (!cam_verify_dma_jpeg_end(frame, dma_len.eof_node_len)) {
        ESP_LOGW(TAG, "JPEG DMA reject: EOI missing len=%u eof_node=%u",
                 (unsigned)frame->len, (unsigned)dma_len.eof_node_len);
        cam_obj->frames[frame_pos].en = 1;
        return false;
    }

    return cam_queue_frame(frame, frame_pos);
}

static void cam_abort_dma_frame(int frame_pos, const char *reason)
{
    ESP_LOGW(TAG, "JPEG DMA abort: %s", reason);
    ll_cam_stop(cam_obj);
    cam_obj->frames[frame_pos].en = 1;
}

static void cam_start_next_or_idle(int *frame_pos)
{
    if (cam_start_frame(frame_pos)) {
        cam_obj->frames[*frame_pos].fb.len = 0;
        cam_obj->state = CAM_STATE_READ_BUF;
    } else {
        cam_obj->state = CAM_STATE_IDLE;
    }
}

static void cam_task_handle_dma_jpeg_event(cam_event_t cam_event, int *frame_pos)
{
    static uint32_t dma_jpeg_vsync_events = 0;
    static uint32_t dma_jpeg_eof_events = 0;
    static uint32_t dma_jpeg_error_events = 0;

    if (cam_event == CAM_DMA_ERROR_EVENT) {
        dma_jpeg_error_events++;
        cam_abort_dma_frame(*frame_pos, "DMA descriptor error");
        cam_start_next_or_idle(frame_pos);
        return;
    }

    if (cam_event == CAM_DMA_FIFO_FULL_EVENT) {
        dma_jpeg_error_events++;
        ESP_LOGW(TAG, "JPEG DMA FIFO full watermark: frame=%d, vsync=%u, eof=%u, err=%u",
                 *frame_pos, (unsigned)dma_jpeg_vsync_events,
                 (unsigned)dma_jpeg_eof_events,
                 (unsigned)dma_jpeg_error_events);
        ll_cam_dma_print_timeout_state(cam_obj);
        cam_abort_dma_frame(*frame_pos, "DMA FIFO full");
        vTaskDelay(pdMS_TO_TICKS(20));
        cam_start_next_or_idle(frame_pos);
        return;
    }

    if (cam_event == CAM_VSYNC_EVENT) {
        dma_jpeg_vsync_events++;
        /* In VSYNC-EOF mode this boundary is expected to generate the GDMA EOF
         * event.  The frame is completed only after descriptor EOF writeback is
         * observed in CAM_IN_SUC_EOF_EVENT. */
        return;
    }

    if (cam_event != CAM_IN_SUC_EOF_EVENT) {
        return;
    }

    dma_jpeg_eof_events++;
    (void)cam_finish_dma_jpeg_frame(*frame_pos);
    cam_start_next_or_idle(frame_pos);
}

static void cam_task_handle_dma_raw_event(cam_event_t cam_event, int *frame_pos, int *cnt)
{
    camera_fb_t *frame = &cam_obj->frames[*frame_pos].fb;

    if (cam_event == CAM_IN_SUC_EOF_EVENT) {
        if ((uint32_t)(*cnt + 1) >= cam_obj->frame_copy_cnt) {
            ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: DMA overflow\r\n"));
            ll_cam_stop(cam_obj);
            cam_obj->frames[*frame_pos].en = 1;
            cam_obj->state = CAM_STATE_IDLE;
            *cnt = 0;
            return;
        }
        (*cnt)++;
        return;
    }

    if (cam_event == CAM_DMA_ERROR_EVENT) {
        cam_abort_dma_frame(*frame_pos, "DMA descriptor error");
        cam_obj->state = CAM_STATE_IDLE;
        *cnt = 0;
        return;
    }

    if (cam_event != CAM_VSYNC_EVENT) {
        return;
    }

    ll_cam_stop(cam_obj);
    frame->len = cam_obj->recv_size;

    (void)cam_queue_frame(frame, *frame_pos);

    cam_start_next_or_idle(frame_pos);
    *cnt = 0;
}

static void cam_task_handle_bounce_eof(camera_fb_t *frame, int *cnt)
{
    size_t pixels_per_dma = (cam_obj->dma_half_buffer_size * cam_obj->fb_bytes_per_pixel) /
                            (cam_obj->dma_bytes_per_item * cam_obj->in_bytes_per_pixel);

    if (cam_obj->fb_size < (frame->len + pixels_per_dma)) {
        ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: FB-OVF\r\n"));
        ll_cam_stop(cam_obj);
        return;
    }

    frame->len += ll_cam_memcpy(cam_obj,
                                &frame->buf[frame->len],
                                &cam_obj->dma_buffer[(*cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size],
                                cam_obj->dma_half_buffer_size);

    if (cam_obj->jpeg_mode && *cnt == 0) {
        int soi_off = cam_verify_jpeg_soi(frame->buf, frame->len);
        if (soi_off != 0) {
            static uint16_t warn_soi_bad_cnt = 0;
            CAM_WARN_THROTTLE(warn_soi_bad_cnt,
                              soi_off > 0 ? "NO-SOI - JPEG start marker not at pos 0"
                                          : "NO-SOI - JPEG start marker missing");
            ll_cam_stop(cam_obj);
            return;
        }
    }

    (*cnt)++;
}

static void cam_task_handle_bounce_vsync(camera_fb_t *frame, int *frame_pos, int *cnt)
{
    size_t pixels_per_dma = (cam_obj->dma_half_buffer_size * cam_obj->fb_bytes_per_pixel) /
                            (cam_obj->dma_bytes_per_item * cam_obj->in_bytes_per_pixel);

    ll_cam_stop(cam_obj);

    if (*cnt || !cam_obj->jpeg_mode) {
        if (cam_obj->jpeg_mode) {
            if (cam_obj->fb_size < (frame->len + pixels_per_dma)) {
                ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: FB-OVF\r\n"));
            } else {
                frame->len += ll_cam_memcpy(cam_obj,
                                            &frame->buf[frame->len],
                                            &cam_obj->dma_buffer[(*cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size],
                                            cam_obj->dma_half_buffer_size);
            }
        } else if (frame->len != cam_obj->fb_size) {
            cam_obj->frames[*frame_pos].en = 1;
            ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: FB-SIZE: %u != %u\r\n"),
                                  frame->len, (unsigned)cam_obj->fb_size);
        }

        if (!cam_obj->frames[*frame_pos].en) {
            (void)cam_queue_frame(frame, *frame_pos);
        }
    }

    cam_start_next_or_idle(frame_pos);
    *cnt = 0;
}

/* Camera task event model:
 *
 * - Bounce mode keeps the historical 1024-byte EOF stream.  EOF copies one
 *   chunk from internal DMA memory, and VSYNC publishes the completed frame.
 *
 * - Raw DMA mode still uses 1024-byte EOF progress events and VSYNC publishes
 *   a recv_size frame.  Pixel-format conversion, when needed, is performed by
 *   cam_take() after cache synchronization.
 *
 * - JPEG DMA mode uses LCD_CAM CAM_VS_EOF_EN.  VSYNC causes one GDMA EOF for
 *   the whole frame.  The task publishes only from the GDMA EOF event, after
 *   descriptor length/eof writeback is visible.  Descriptor-empty/error is the
 *   overrun guard; there is no circular-descriptor overshoot area in this mode.
 *
 * When all framebuffer slots are held by the application, the task becomes
 * idle.  cam_give() posts CAM_FRAME_RETURNED_EVENT after marking a slot free so
 * the task can resume capture without depending on LCD_CAM VSYNC interrupts
 * while the peripheral is stopped.
 */
static void cam_task(void *arg)
{
    int cnt = 0;
    int frame_pos = 0;
    cam_event_t cam_event = 0;

    cam_obj->state = CAM_STATE_IDLE;
    xQueueReset(cam_obj->event_queue);

    while (1) {
        xQueueReceive(cam_obj->event_queue, (void *)&cam_event, portMAX_DELAY);
        DBG_PIN_SET(1);

        switch (cam_obj->state) {
        case CAM_STATE_IDLE:
            if (cam_event == CAM_VSYNC_EVENT || cam_event == CAM_FRAME_RETURNED_EVENT) {
                cam_start_next_or_idle(&frame_pos);
                cnt = 0;
            }
            break;

        case CAM_STATE_READ_BUF: {
            camera_fb_t *frame = &cam_obj->frames[frame_pos].fb;

            if (cam_obj->dma_mode && cam_obj->jpeg_mode) {
                cam_task_handle_dma_jpeg_event(cam_event, &frame_pos);
                cnt = 0;
                break;
            }

            if (cam_obj->dma_mode) {
                cam_task_handle_dma_raw_event(cam_event, &frame_pos, &cnt);
                break;
            }

            if (cam_event == CAM_IN_SUC_EOF_EVENT) {
                cam_task_handle_bounce_eof(frame, &cnt);
            } else if (cam_event == CAM_VSYNC_EVENT) {
                cam_task_handle_bounce_vsync(frame, &frame_pos, &cnt);
            } else if (cam_event == CAM_DMA_ERROR_EVENT) {
                ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: unexpected DMA error in bounce mode\r\n"));
                ll_cam_stop(cam_obj);
                cam_obj->state = CAM_STATE_IDLE;
            }
            break;
        }
        }

        DBG_PIN_SET(0);
    }
}


static lldesc_t * allocate_dma_descriptors(uint32_t count, uint16_t size, uint8_t * buffer, bool circular)
{
    lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (dma == NULL) {
        return dma;
    }

    for (int x = 0; x < count; x++) {
        dma[x].size = size;
        dma[x].length = 0;
        dma[x].sosf = 0;
        dma[x].eof = 0;
        dma[x].owner = 1;
        dma[x].buf = (buffer + size * x);
        dma[x].empty = circular || (x + 1 < count) ? (uint32_t)&dma[(x + 1) % count] : 0;
    }
    return dma;
}

static esp_err_t cam_dma_config(const camera_config_t *config)
{
    bool ret = ll_cam_dma_sizes(cam_obj);
    if (0 == ret) {
        return ESP_FAIL;
    }

    cam_obj->dma_node_cnt = (cam_obj->dma_buffer_size) / cam_obj->dma_node_buffer_size; // Number of DMA nodes
    cam_obj->frame_copy_cnt = cam_obj->recv_size / cam_obj->dma_half_buffer_size; // Number of interrupted copies, ping-pong copy
    if (cam_obj->dma_mode && !cam_obj->jpeg_mode) {
        cam_obj->frame_copy_cnt++;
    }

    ESP_LOGI(TAG, "buffer_size: %d, half_buffer_size: %d, node_buffer_size: %d, node_cnt: %d, total_cnt: %d",
             (int) cam_obj->dma_buffer_size, (int) cam_obj->dma_half_buffer_size, (int) cam_obj->dma_node_buffer_size,
             (int) cam_obj->dma_node_cnt, (int) cam_obj->frame_copy_cnt);

    cam_obj->dma_buffer = NULL;
    cam_obj->dma = NULL;

    cam_obj->frames = (cam_frame_t *)heap_caps_aligned_calloc(alignof(cam_frame_t), 1, cam_obj->frame_cnt * sizeof(cam_frame_t), MALLOC_CAP_DEFAULT);
    CAM_CHECK(cam_obj->frames != NULL, "frames malloc failed", ESP_FAIL);

    uint8_t dma_align = 0;
    size_t fb_size = cam_obj->fb_size;
    if (cam_obj->dma_mode) {
        dma_align = ll_cam_get_dma_align(cam_obj);
        if (fb_size < cam_obj->dma_buffer_size) {
            fb_size = cam_obj->dma_buffer_size;
        }
    }

    /* Allocate memory for frame buffer.  DMA-mode frame buffers are read by
     * the CPU after external-memory DMA writes, so align the allocation to at
     * least the data-cache line size.  This keeps cache invalidation from
     * sharing the first line of the framebuffer with unrelated heap data. */
    size_t fb_align = 16;
    if (cam_obj->dma_mode) {
        fb_align = esp_camera_dcache_line_size();
        if (fb_align == 0) {
            fb_align = 32;
        }
        if (fb_align < dma_align) {
            fb_align = dma_align;
        }
    }

    size_t alloc_size = fb_size * sizeof(uint8_t) + dma_align;
    uint32_t _caps = MALLOC_CAP_8BIT;
    if (CAMERA_FB_IN_DRAM == config->fb_location) {
        _caps |= MALLOC_CAP_INTERNAL;
    } else {
        _caps |= MALLOC_CAP_SPIRAM;
    }
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        cam_obj->frames[x].dma = NULL;
        cam_obj->frames[x].fb_offset = 0;
        cam_obj->frames[x].en = 0;
        ESP_LOGI(TAG, "Allocating %d Byte frame buffer in %s", alloc_size, _caps & MALLOC_CAP_SPIRAM ? "PSRAM" : "OnBoard RAM");
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
        // In IDF v4.2 and earlier, memory returned by heap_caps_aligned_alloc must be freed using heap_caps_aligned_free.
        // And heap_caps_aligned_free is deprecated on v4.3.
        cam_obj->frames[x].fb.buf = (uint8_t *)heap_caps_aligned_alloc(fb_align, alloc_size, _caps);
#else
        cam_obj->frames[x].fb.buf = (uint8_t *)heap_caps_malloc(alloc_size, _caps);
#endif
        CAM_CHECK(cam_obj->frames[x].fb.buf != NULL, "frame buffer malloc failed", ESP_FAIL);
        if (cam_obj->dma_mode) {
            uintptr_t mis = (uintptr_t)cam_obj->frames[x].fb.buf & (dma_align - 1);
            cam_obj->frames[x].fb_offset = (dma_align - mis) & (dma_align - 1);
            cam_obj->frames[x].fb.buf += cam_obj->frames[x].fb_offset;
            ESP_LOGI(TAG, "Frame[%d]: Offset: %u, Addr: 0x%08X", x, cam_obj->frames[x].fb_offset, (unsigned)cam_obj->frames[x].fb.buf);
            cam_obj->frames[x].dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->frames[x].fb.buf, !(cam_obj->jpeg_mode && cam_obj->dma_mode));
            CAM_CHECK(cam_obj->frames[x].dma != NULL, "frame dma malloc failed", ESP_FAIL);
        }
        cam_obj->frames[x].en = 1;
    }

    if (!cam_obj->dma_mode) {
        cam_obj->dma_buffer = (uint8_t *)heap_caps_malloc(cam_obj->dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
        if(NULL == cam_obj->dma_buffer) {
            ESP_LOGE(TAG,"%s(%d): DMA buffer %d Byte malloc failed, the current largest free block:%d Byte", __FUNCTION__, __LINE__,
                     (int) cam_obj->dma_buffer_size, (int) heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            return ESP_FAIL;
        }

        cam_obj->dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->dma_buffer, true);
        CAM_CHECK(cam_obj->dma != NULL, "dma malloc failed", ESP_FAIL);
    }

    return ESP_OK;
}

esp_err_t cam_init(const camera_config_t *config)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);

    esp_err_t ret = ESP_OK;
    cam_obj = (cam_obj_t *)heap_caps_calloc(1, sizeof(cam_obj_t), MALLOC_CAP_DMA);
    CAM_CHECK(NULL != cam_obj, "lcd_cam object malloc error", ESP_ERR_NO_MEM);

    cam_obj->swap_data = 0;
    cam_obj->vsync_pin = config->pin_vsync;
    cam_obj->vsync_invert = true;

    ll_cam_set_pin(cam_obj, config);
    ret = ll_cam_config(cam_obj, config);
    CAM_CHECK_GOTO(ret == ESP_OK, "ll_cam initialize failed", err);

#if CAMERA_DBG_PIN_ENABLE
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[DBG_PIN_NUM], PIN_FUNC_GPIO);
    gpio_set_direction(DBG_PIN_NUM, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(DBG_PIN_NUM, GPIO_FLOATING);
#endif

    ESP_LOGI(TAG, "cam init ok");
    return ESP_OK;

err:
    free(cam_obj);
    cam_obj = NULL;
    return ESP_FAIL;
}

esp_err_t cam_config(const camera_config_t *config, framesize_t frame_size, uint16_t sensor_pid)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    ret = ll_cam_set_sample_mode(cam_obj, (pixformat_t)config->pixel_format, config->xclk_freq_hz, sensor_pid);
    CAM_CHECK_GOTO(ret == ESP_OK, "ll_cam_set_sample_mode failed", err);
    
    cam_obj->jpeg_mode = config->pixel_format == PIXFORMAT_JPEG;
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
    cam_obj->dma_mode = g_dma_mode;
#else
    cam_obj->dma_mode = false;
#endif
    ESP_LOGI(TAG, "DMA mode %s", cam_obj->dma_mode ? "enabled" : "disabled");
    cam_obj->frame_cnt = config->fb_count;
    cam_obj->width = resolution[frame_size].width;
    cam_obj->height = resolution[frame_size].height;

    if(cam_obj->jpeg_mode){
#ifdef CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE_AUTO
        cam_obj->recv_size = cam_obj->width * cam_obj->height / 5;
#else
        cam_obj->recv_size = CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE;
#endif
        cam_obj->fb_size = cam_obj->recv_size;
    } else {
        cam_obj->recv_size = cam_obj->width * cam_obj->height * cam_obj->in_bytes_per_pixel;
        cam_obj->fb_size = cam_obj->width * cam_obj->height * cam_obj->fb_bytes_per_pixel;
    }

    ret = cam_dma_config(config);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam_dma_config failed", err);

    size_t queue_size = cam_obj->dma_half_buffer_cnt - 1;
    if (queue_size == 0) {
        queue_size = 1;
    }
    cam_obj->event_queue = xQueueCreate(queue_size, sizeof(cam_event_t));
    CAM_CHECK_GOTO(cam_obj->event_queue != NULL, "event_queue create failed", err);

    size_t frame_buffer_queue_len = cam_obj->frame_cnt;
    if (config->grab_mode == CAMERA_GRAB_LATEST && cam_obj->frame_cnt > 1) {
        frame_buffer_queue_len = cam_obj->frame_cnt - 1;
    }
    cam_obj->frame_buffer_queue = xQueueCreate(frame_buffer_queue_len, sizeof(camera_fb_t*));
    CAM_CHECK_GOTO(cam_obj->frame_buffer_queue != NULL, "frame_buffer_queue create failed", err);

    ret = ll_cam_init_isr(cam_obj);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam intr alloc failed", err);


#if CONFIG_CAMERA_CORE0
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle, 0);
#elif CONFIG_CAMERA_CORE1
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle, 1);
#else
    xTaskCreate(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle);
#endif

    ESP_LOGI(TAG, "cam config ok");
    return ESP_OK;

err:
    cam_deinit();
    return ESP_FAIL;
}

esp_err_t cam_deinit(void)
{
    if (!cam_obj) {
        return ESP_FAIL;
    }

    cam_stop();
    if (cam_obj->task_handle) {
        vTaskDelete(cam_obj->task_handle);
    }
    if (cam_obj->event_queue) {
        vQueueDelete(cam_obj->event_queue);
    }
    if (cam_obj->frame_buffer_queue) {
        vQueueDelete(cam_obj->frame_buffer_queue);
    }

    ll_cam_deinit(cam_obj);

    if (cam_obj->dma) {
        free(cam_obj->dma);
    }
    if (cam_obj->dma_buffer) {
        free(cam_obj->dma_buffer);
    }
    if (cam_obj->frames) {
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            free(cam_obj->frames[x].fb.buf - cam_obj->frames[x].fb_offset);
            if (cam_obj->frames[x].dma) {
                free(cam_obj->frames[x].dma);
            }
        }
        free(cam_obj->frames);
    }

    free(cam_obj);
    cam_obj = NULL;
    return ESP_OK;
}

void cam_stop(void)
{
    ll_cam_vsync_intr_enable(cam_obj, false);
    ll_cam_stop(cam_obj);
}

void cam_start(void)
{
    ll_cam_vsync_intr_enable(cam_obj, true);
}

camera_fb_t *cam_take(TickType_t timeout)
{
    camera_fb_t *dma_buffer = NULL;
    const TickType_t start = xTaskGetTickCount();
#if CONFIG_IDF_TARGET_ESP32S3
    uint16_t dma_reset_counter = 0;
    static const uint8_t MAX_GDMA_RESETS = 3;
#else
    /* throttle repeated NULL frame warnings */
    static uint16_t warn_null_cnt = 0;
#endif
    /* throttle repeated NO-EOI warnings */
    static uint16_t warn_eoi_miss_cnt = 0;

    for (;;)
    {
        TickType_t elapsed = xTaskGetTickCount() - start; /* TickType_t is unsigned so rollover is safe */
        if (elapsed >= timeout) {
            ESP_LOGW(TAG, "Failed to get frame: timeout");
#if CONFIG_IDF_TARGET_ESP32S3
            /* Keep cam_take() lightweight; this runs in the framebuffer consumer
             * task, which may have a smaller stack than the camera task. */
            if (cam_obj->dma_mode && cam_obj->jpeg_mode) {
                static uint16_t timeout_dump_cnt = 0;
                if ((timeout_dump_cnt++ & 0x03) == 0) {
                    ll_cam_dma_print_timeout_state(cam_obj);
                }
            }
#endif
            return NULL;
        }
        TickType_t remaining = timeout - elapsed;

        if (xQueueReceive(cam_obj->frame_buffer_queue, (void *)&dma_buffer, remaining) == pdFALSE) {
            continue;
        }


        if (!dma_buffer) {
            /* Work-around for ESP32-S3 GDMA freeze when Wi-Fi STA starts.
             * See esp32-camera commit 984999f (issue #620). */
#if CONFIG_IDF_TARGET_ESP32S3
            if (dma_reset_counter < MAX_GDMA_RESETS) {
                ll_cam_dma_reset(cam_obj);
                dma_reset_counter++;
                continue; /* retry with queue timeout */
            }
            if (dma_reset_counter == MAX_GDMA_RESETS) {
                ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: Giving up GDMA reset after %u tries\r\n"),
                                     (unsigned) dma_reset_counter);
                dma_reset_counter++; /* suppress further logs */
            }
#else
            /* Early warning for misbehaving sensors on other chips */
            CAM_WARN_THROTTLE(warn_null_cnt,
                              "Unexpected NULL frame on " CONFIG_IDF_TARGET);
#endif
            vTaskDelay(1); /* immediate yield once resets are done */
            continue;             /* go to top of loop */
        }

        if (cam_obj->jpeg_mode) {
            /* find the end marker for JPEG. Data after that can be discarded */
            int offset_e = -1;
            if (cam_obj->dma_mode) {
                /* DMA-mode JPEGs are SOI/EOI validated and, if needed, trimmed
                 * before entering the frame queue. */
                return dma_buffer;
            } else {
                offset_e = cam_verify_jpeg_eoi(dma_buffer->buf, dma_buffer->len, false);
            }

            if (offset_e >= 0) {
                dma_buffer->len = offset_e + JPEG_EOI_MARKER_LEN;
                if (cam_obj->dma_mode) {
                    /* DMA may bypass cache, ensure full frame is visible */
                    cam_drop_psram_cache(dma_buffer->buf, dma_buffer->len);
                }
                return dma_buffer;
            }

skip_eoi_check:

            CAM_WARN_THROTTLE(warn_eoi_miss_cnt,
                              "NO-EOI - JPEG end marker missing");
            cam_give(dma_buffer);
            continue; /* wait for another frame */
        } else if (cam_obj->dma_mode &&
                   cam_obj->in_bytes_per_pixel != cam_obj->fb_bytes_per_pixel) {
            /* currently used only for YUV to GRAYSCALE */
            dma_buffer->len = ll_cam_memcpy(cam_obj, dma_buffer->buf, dma_buffer->buf, dma_buffer->len);
        }

        if (cam_obj->dma_mode) {
            /* DMA may bypass cache, ensure full frame is visible to the app */
            cam_drop_psram_cache(dma_buffer->buf, dma_buffer->len);
        }

        return dma_buffer;
    }
}

void cam_give(camera_fb_t *dma_buffer)
{
    int returned_pos = -1;

    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        if (&cam_obj->frames[x].fb == dma_buffer) {
            cam_obj->frames[x].en = 1;
            returned_pos = x;
            break;
        }
    }

    if (returned_pos >= 0 && cam_obj->event_queue) {
        cam_event_t event = CAM_FRAME_RETURNED_EVENT;
        (void)xQueueSend(cam_obj->event_queue, (void *)&event, 0);
    }
}

void cam_give_all(void) {
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        cam_obj->frames[x].en = 1;
    }
}

bool cam_get_available_frames(void)
{
    return 0 < uxQueueMessagesWaiting(cam_obj->frame_buffer_queue);
}

void cam_set_dma_mode(bool enable)
{
    portENTER_CRITICAL(&g_dma_mode_lock);
    g_dma_mode = enable;
    portEXIT_CRITICAL(&g_dma_mode_lock);
}

bool cam_get_dma_mode(void)
{
    return g_dma_mode;
}
