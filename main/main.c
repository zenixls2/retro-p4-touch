#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ppa.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7701.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_lvgl_port.h"
#include "esp_lvgl_port_touch.h"
#include "lvgl.h"
#include "miniz.h"

#include "common.h"
#include "cpu.h"
#include "gba_memory.h"
#include "input.h"
#include "main.h"
#include "serial.h"
#include "sound.h"
#include "video.h"

static const char *TAG = "retro_touch";

#define LCD_H_RES 480
#define LCD_V_RES 800

#define LCD_MIPI_DSI_LANE_BITRATE_MBPS 500

#define MIPI_DSI_PHY_PWR_LDO_CHAN 3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500

#define PIN_NUM_LCD_RST GPIO_NUM_5
#define PIN_NUM_BK_LIGHT GPIO_NUM_23

#define LCD_TOUCH_RST GPIO_NUM_NC
#define LCD_TOUCH_INT GPIO_NUM_NC
#define TOUCH_I2C_SDA_PIN GPIO_NUM_7
#define TOUCH_I2C_SCL_PIN GPIO_NUM_8

#define LCD_BK_LIGHT_ON_LEVEL 1
#define LCD_BK_LIGHT_OFF_LEVEL (!LCD_BK_LIGHT_ON_LEVEL)

#define GBA_FRAME_WIDTH GBA_SCREEN_WIDTH
#define GBA_FRAME_HEIGHT GBA_SCREEN_HEIGHT
#define GBA_FRAME_PIXELS (GBA_FRAME_WIDTH * GBA_FRAME_HEIGHT)
#define GBA_VIEWPORT_WIDTH 414
#define GBA_VIEWPORT_HEIGHT 292
#define GBA_VIEWPORT_BORDER_WIDTH 0
#define GBA_CANVAS_WIDTH (GBA_VIEWPORT_WIDTH - (GBA_VIEWPORT_BORDER_WIDTH * 2))
#define GBA_CANVAS_HEIGHT (GBA_VIEWPORT_HEIGHT - (GBA_VIEWPORT_BORDER_WIDTH * 2))
#define GBA_CANVAS_PIXELS (GBA_CANVAS_WIDTH * GBA_CANVAS_HEIGHT)
#define GBA_CANVAS_BYTES (GBA_CANVAS_PIXELS * sizeof(uint16_t))
#define GBA_PPA_BUF_ALIGN CONFIG_CACHE_L2_CACHE_LINE_SIZE
#define GBA_PPA_SCALE_FRAG_STEPS 16U
#define GBA_TARGET_FRAME_TIME_MS 16
#define GBA_FRAME_STREAM_STATUS_MS 5000
#define GBA_ROM_MAX_BYTES (32U * 1024U * 1024U)

typedef struct {
    const char *label;
    uint32_t button_mask;
} gba_button_t;

static const gba_button_t BTN_UP = {LV_SYMBOL_UP, BUTTON_UP};
static const gba_button_t BTN_DOWN = {LV_SYMBOL_DOWN, BUTTON_DOWN};
static const gba_button_t BTN_LEFT = {LV_SYMBOL_LEFT, BUTTON_LEFT};
static const gba_button_t BTN_RIGHT = {LV_SYMBOL_RIGHT, BUTTON_RIGHT};
static const gba_button_t BTN_A = {"A", BUTTON_A};
static const gba_button_t BTN_B = {"B", BUTTON_B};
static const gba_button_t BTN_L = {"L", BUTTON_L};
static const gba_button_t BTN_R = {"R", BUTTON_R};
static const gba_button_t BTN_START = {"START", BUTTON_START};
static const gba_button_t BTN_SELECT = {"SELECT", BUTTON_SELECT};
static const gba_button_t BTN_POWER = {LV_SYMBOL_POWER, 0};

static lv_obj_t *s_game_canvas;
static lv_obj_t *s_status_label;
static lv_timer_t *s_loading_timer;

static uint16_t *s_game_canvas_buf;
static size_t s_game_canvas_buf_size;
static uint16_t *s_gp_frame_buf;
static uint8_t *s_rom_buffer;
static size_t s_rom_size;
static ppa_client_handle_t s_ppa_srm_client;
static bool s_ppa_srm_ready;
static bool s_ppa_srm_warned;

static volatile uint32_t s_gp_pressed_mask;
static volatile bool s_gba_loading;
static volatile bool s_gba_running;
static volatile bool s_gba_frame_ready;
static bool s_emu_task_started;
static bool s_gp_core_initialized;
static uint8_t s_loading_anim_phase;
static uint32_t s_last_frame_counter;
static esp_err_t s_gba_init_result = ESP_FAIL;
static TickType_t s_frame_stream_status_deadline;
static uint16_t s_scale_x_lut[GBA_CANVAS_WIDTH];
static uint16_t s_scale_y_lut[GBA_CANVAS_HEIGHT];
static bool s_scale_lut_ready;
static bool s_layout_debug_logged;
static bool s_ppa_debug_logged;

static void gba_prepare_scale_lut(void);

extern const uint8_t pokemon_gba_zlib_start[] asm("_binary_pokemon_gba_zlib_start");
extern const uint8_t pokemon_gba_zlib_end[] asm("_binary_pokemon_gba_zlib_end");
extern const uint8_t open_gba_bios_bin_start[] asm("_binary_open_gba_bios_bin_start");
extern const uint8_t open_gba_bios_bin_end[] asm("_binary_open_gba_bios_bin_end");
extern void set_gamepak_source_memory(const u8 *data, u32 size);

u32 idle_loop_target_pc = 0xFFFFFFFF;
u32 translation_gate_target_pc[MAX_TRANSLATION_GATES];
u32 translation_gate_targets = 0;
boot_mode selected_boot_mode = boot_game;

u32 skip_next_frame = 0;
int sprite_limit = 1;

void netpacket_poll_receive(void)
{
}

void netpacket_send(uint16_t client_id, const void *buf, size_t len)
{
    (void)client_id;
    (void)buf;
    (void)len;
}

void set_fastforward_override(bool fastforward)
{
    (void)fastforward;
}

static size_t gba_align_up(size_t value, size_t alignment)
{
    return ((value + alignment - 1U) / alignment) * alignment;
}

static bool gba_is_aligned(const void *ptr, size_t alignment)
{
    return (((uintptr_t)ptr) % alignment) == 0U;
}

static bool gba_tick_reached(TickType_t now, TickType_t deadline)
{
    return ((int32_t)(now - deadline) >= 0);
}

static uint32_t gba_ppa_scale_steps(uint32_t src_size, uint32_t dst_size)
{
    uint32_t steps = (dst_size * GBA_PPA_SCALE_FRAG_STEPS) / src_size;
    if (steps < GBA_PPA_SCALE_FRAG_STEPS) {
        steps = GBA_PPA_SCALE_FRAG_STEPS;
    }
    return steps;
}

static void gba_fill_ppa_uncovered_edges(uint32_t ppa_out_w, uint32_t ppa_out_h)
{
    if ((s_gp_frame_buf == NULL) || (s_game_canvas_buf == NULL)) {
        return;
    }

    if (ppa_out_w > GBA_CANVAS_WIDTH) {
        ppa_out_w = GBA_CANVAS_WIDTH;
    }
    if (ppa_out_h > GBA_CANVAS_HEIGHT) {
        ppa_out_h = GBA_CANVAS_HEIGHT;
    }

    if ((ppa_out_w >= GBA_CANVAS_WIDTH) && (ppa_out_h >= GBA_CANVAS_HEIGHT)) {
        return;
    }

    gba_prepare_scale_lut();

    if (ppa_out_w < GBA_CANVAS_WIDTH) {
        for (uint32_t y = 0; y < ppa_out_h; y++) {
            const uint16_t *src_row = s_gp_frame_buf + ((uint32_t)s_scale_y_lut[y] * GBA_FRAME_WIDTH);
            uint16_t *dst_row = s_game_canvas_buf + ((uint32_t)y * GBA_CANVAS_WIDTH);
            for (uint32_t x = ppa_out_w; x < GBA_CANVAS_WIDTH; x++) {
                dst_row[x] = src_row[s_scale_x_lut[x]];
            }
        }
    }

    if (ppa_out_h < GBA_CANVAS_HEIGHT) {
        for (uint32_t y = ppa_out_h; y < GBA_CANVAS_HEIGHT; y++) {
            const uint16_t *src_row = s_gp_frame_buf + ((uint32_t)s_scale_y_lut[y] * GBA_FRAME_WIDTH);
            uint16_t *dst_row = s_game_canvas_buf + ((uint32_t)y * GBA_CANVAS_WIDTH);
            for (uint32_t x = 0; x < GBA_CANVAS_WIDTH; x++) {
                dst_row[x] = src_row[s_scale_x_lut[x]];
            }
        }
    }
}

static uint16_t *gba_alloc_canvas_buffer(size_t *out_size)
{
    const size_t aligned_size = gba_align_up(GBA_CANVAS_BYTES, GBA_PPA_BUF_ALIGN);
    uint16_t *buf = heap_caps_aligned_calloc(GBA_PPA_BUF_ALIGN,
                                             aligned_size,
                                             1,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL) {
        buf = heap_caps_aligned_calloc(GBA_PPA_BUF_ALIGN,
                                       aligned_size,
                                       1,
                                       MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (buf == NULL) {
        buf = heap_caps_calloc(GBA_CANVAS_PIXELS, sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (buf == NULL) {
        buf = heap_caps_calloc(GBA_CANVAS_PIXELS, sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }

    if (buf != NULL) {
        if (out_size != NULL) {
            *out_size = gba_is_aligned(buf, GBA_PPA_BUF_ALIGN) ? aligned_size : GBA_CANVAS_BYTES;
        }
    }

    return buf;
}

static esp_err_t gba_ensure_canvas_buffer(void)
{
    if (s_game_canvas_buf != NULL) {
        if (s_game_canvas_buf_size == 0U) {
            s_game_canvas_buf_size = GBA_CANVAS_BYTES;
        }
        return ESP_OK;
    }

    s_game_canvas_buf = gba_alloc_canvas_buffer(&s_game_canvas_buf_size);
    if (s_game_canvas_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static esp_err_t gba_init_ppa_client(void)
{
    if (s_ppa_srm_ready && (s_ppa_srm_client != NULL)) {
        return ESP_OK;
    }

    const ppa_client_config_t ppa_cfg = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };

    esp_err_t err = ppa_register_client(&ppa_cfg, &s_ppa_srm_client);
    if (err != ESP_OK) {
        return err;
    }

    s_ppa_srm_ready = true;
    return ESP_OK;
}

static bool gba_scale_frame_with_ppa(void)
{
    if ((!s_ppa_srm_ready) || (s_ppa_srm_client == NULL)) {
        return false;
    }
    if ((s_gp_frame_buf == NULL) || (s_game_canvas_buf == NULL)) {
        return false;
    }
    if (!gba_is_aligned(s_game_canvas_buf, GBA_PPA_BUF_ALIGN)) {
        return false;
    }
    if ((s_game_canvas_buf_size == 0U) || ((s_game_canvas_buf_size % GBA_PPA_BUF_ALIGN) != 0U)) {
        return false;
    }

    const uint32_t scale_x_steps = gba_ppa_scale_steps(GBA_FRAME_WIDTH, GBA_CANVAS_WIDTH);
    const uint32_t scale_y_steps = gba_ppa_scale_steps(GBA_FRAME_HEIGHT, GBA_CANVAS_HEIGHT);
    const uint32_t ppa_out_w = (GBA_FRAME_WIDTH * scale_x_steps) / GBA_PPA_SCALE_FRAG_STEPS;
    const uint32_t ppa_out_h = (GBA_FRAME_HEIGHT * scale_y_steps) / GBA_PPA_SCALE_FRAG_STEPS;
    const uint32_t gap_w = (ppa_out_w < GBA_CANVAS_WIDTH) ? (GBA_CANVAS_WIDTH - ppa_out_w) : 0;
    const uint32_t gap_h = (ppa_out_h < GBA_CANVAS_HEIGHT) ? (GBA_CANVAS_HEIGHT - ppa_out_h) : 0;

    const ppa_srm_oper_config_t srm_cfg = {
        .in = {
            .buffer = s_gp_frame_buf,
            .pic_w = GBA_FRAME_WIDTH,
            .pic_h = GBA_FRAME_HEIGHT,
            .block_w = GBA_FRAME_WIDTH,
            .block_h = GBA_FRAME_HEIGHT,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = s_game_canvas_buf,
            .buffer_size = s_game_canvas_buf_size,
            .pic_w = GBA_CANVAS_WIDTH,
            .pic_h = GBA_CANVAS_HEIGHT,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = (float)scale_x_steps / (float)GBA_PPA_SCALE_FRAG_STEPS,
        .scale_y = (float)scale_y_steps / (float)GBA_PPA_SCALE_FRAG_STEPS,
        .mirror_x = false,
        .mirror_y = false,
        .rgb_swap = false,
        .byte_swap = false,
        .alpha_update_mode = PPA_ALPHA_NO_CHANGE,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    esp_err_t err = ppa_do_scale_rotate_mirror(s_ppa_srm_client, &srm_cfg);
    if (err != ESP_OK) {
        if (!s_ppa_srm_warned) {
            ESP_LOGW(TAG, "PPA scaler unavailable (%s), fallback to CPU nearest-neighbor", esp_err_to_name(err));
            s_ppa_srm_warned = true;
        }
        return false;
    }

    if (!s_ppa_debug_logged) {
        ESP_LOGI(TAG,
                 "PPA dbg src=%ux%u target=%ux%u steps=%u/%u effective=%ux%u gap=%ux%u",
                 (unsigned int)GBA_FRAME_WIDTH,
                 (unsigned int)GBA_FRAME_HEIGHT,
                 (unsigned int)GBA_CANVAS_WIDTH,
                 (unsigned int)GBA_CANVAS_HEIGHT,
                 (unsigned int)scale_x_steps,
                 (unsigned int)scale_y_steps,
                 (unsigned int)ppa_out_w,
                 (unsigned int)ppa_out_h,
                 (unsigned int)gap_w,
                 (unsigned int)gap_h);
        s_ppa_debug_logged = true;
    }

    if ((gap_w > 0U) || (gap_h > 0U)) {
        gba_fill_ppa_uncovered_edges(ppa_out_w, ppa_out_h);
    }

    return true;
}

static bool gba_source_frame_has_non_zero(void)
{
    if (s_gp_frame_buf == NULL) {
        return false;
    }

    for (uint32_t i = 0; i < GBA_FRAME_PIXELS; i++) {
        if (s_gp_frame_buf[i] != 0U) {
            return true;
        }
    }

    return false;
}

static void gba_prepare_scale_lut(void)
{
    if (s_scale_lut_ready) {
        return;
    }

    for (uint32_t x = 0; x < GBA_CANVAS_WIDTH; x++) {
        uint32_t src_x = (x * GBA_FRAME_WIDTH) / GBA_CANVAS_WIDTH;
        if (src_x >= GBA_FRAME_WIDTH) {
            src_x = GBA_FRAME_WIDTH - 1;
        }
        s_scale_x_lut[x] = (uint16_t)src_x;
    }

    for (uint32_t y = 0; y < GBA_CANVAS_HEIGHT; y++) {
        uint32_t src_y = (y * GBA_FRAME_HEIGHT) / GBA_CANVAS_HEIGHT;
        if (src_y >= GBA_FRAME_HEIGHT) {
            src_y = GBA_FRAME_HEIGHT - 1;
        }
        s_scale_y_lut[y] = (uint16_t)src_y;
    }

    s_scale_lut_ready = true;
}

static const st7701_lcd_init_cmd_t lcd_cmd[] = {
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xEF, (uint8_t []){0x08}, 1, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, (uint8_t []){0x63, 0x00}, 2, 0},
    {0xC1, (uint8_t []){0x0D, 0x02}, 2, 0},
    {0xC2, (uint8_t []){0x10, 0x08}, 2, 0},
    {0xCC, (uint8_t []){0x10}, 1, 0},
    {0xB0, (uint8_t []){0x80, 0x09, 0x53, 0x0C, 0xD0, 0x07, 0x0C, 0x09, 0x09, 0x28, 0x06, 0xD4, 0x13, 0x69, 0x2B, 0x71}, 16, 0},
    {0xB1, (uint8_t []){0x80, 0x94, 0x5A, 0x10, 0xD3, 0x06, 0x0A, 0x08, 0x08, 0x25, 0x03, 0xD3, 0x12, 0x66, 0x6A, 0x0D}, 16, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xB0, (uint8_t []){0x5D}, 1, 0},
    {0xB1, (uint8_t []){0x58}, 1, 0},
    {0xB2, (uint8_t []){0x87}, 1, 0},
    {0xB3, (uint8_t []){0x80}, 1, 0},
    {0xB5, (uint8_t []){0x4E}, 1, 0},
    {0xB7, (uint8_t []){0x85}, 1, 0},
    {0xB8, (uint8_t []){0x21}, 1, 0},
    {0xB9, (uint8_t []){0x10, 0x1F}, 2, 0},
    {0xBB, (uint8_t []){0x03}, 1, 0},
    {0xBC, (uint8_t []){0x00}, 1, 0},
    {0xC1, (uint8_t []){0x78}, 1, 0},
    {0xC2, (uint8_t []){0x78}, 1, 0},
    {0xD0, (uint8_t []){0x88}, 1, 0},
    {0xE0, (uint8_t []){0x00, 0x3A, 0x02}, 3, 0},
    {0xE1, (uint8_t []){0x04, 0xA0, 0x00, 0xA0, 0x05, 0xA0, 0x00, 0xA0, 0x00, 0x40, 0x40}, 11, 0},
    {0xE2, (uint8_t []){0x30, 0x00, 0x40, 0x40, 0x32, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00}, 13, 0},
    {0xE3, (uint8_t []){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE4, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE5, (uint8_t []){0x09, 0x2E, 0xA0, 0xA0, 0x0B, 0x30, 0xA0, 0xA0, 0x05, 0x2A, 0xA0, 0xA0, 0x07, 0x2C, 0xA0, 0xA0}, 16, 0},
    {0xE6, (uint8_t []){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE7, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE8, (uint8_t []){0x08, 0x2D, 0xA0, 0xA0, 0x0A, 0x2F, 0xA0, 0xA0, 0x04, 0x29, 0xA0, 0xA0, 0x06, 0x2B, 0xA0, 0xA0}, 16, 0},
    {0xEB, (uint8_t []){0x00, 0x00, 0x4E, 0x4E, 0x00, 0x00, 0x00}, 7, 0},
    {0xEC, (uint8_t []){0x08, 0x01}, 2, 0},
    {0xED, (uint8_t []){0xB0, 0x2B, 0x98, 0xA4, 0x56, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0x65, 0x4A, 0x89, 0xB2, 0x0B}, 16, 0},
    {0xEF, (uint8_t []){0x08, 0x08, 0x08, 0x45, 0x3F, 0x54}, 6, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
    {0x11, (uint8_t []){0x00}, 1, 120},
    {0x29, (uint8_t []){0x00}, 1, 20},
};

static void gba_emulation_task(void *arg);

static esp_err_t bsp_enable_dsi_phy_power(void)
{
#if MIPI_DSI_PHY_PWR_LDO_CHAN > 0
    static esp_ldo_channel_handle_t phy_pwr_chan;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    esp_err_t err = esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "MIPI DSI PHY powered on");
#endif
    return ESP_OK;
}

static esp_err_t bsp_init_backlight(void)
{
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT,
    };
    esp_err_t err = gpio_config(&bk_gpio_config);
    if (err != ESP_OK) {
        return err;
    }
    return gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
}

static void bsp_set_backlight(bool on)
{
    gpio_set_level(PIN_NUM_BK_LIGHT, on ? LCD_BK_LIGHT_ON_LEVEL : LCD_BK_LIGHT_OFF_LEVEL);
}

static esp_err_t bsp_try_init_touch_with_addr(i2c_master_bus_handle_t i2c_bus,
                                              uint8_t dev_addr,
                                              esp_lcd_touch_handle_t *touch_handle)
{
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.scl_speed_hz = 100000;
    tp_io_config.dev_addr = dev_addr;

    esp_err_t err = esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_config, &tp_io_handle);
    if (err != ESP_OK) {
        return err;
    }

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = LCD_TOUCH_RST,
        .int_gpio_num = LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    err = esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, touch_handle);
    if (err != ESP_OK) {
        esp_lcd_panel_io_del(tp_io_handle);
    }
    return err;
}

static esp_err_t bsp_init_touch(esp_lcd_touch_handle_t *touch_handle)
{
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = TOUCH_I2C_SDA_PIN,
        .scl_io_num = TOUCH_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    esp_err_t err = i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus);
    if (err != ESP_OK) {
        return err;
    }

    err = bsp_try_init_touch_with_addr(i2c_bus, ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS, touch_handle);
    if (err == ESP_OK) {
        return ESP_OK;
    }

    ESP_LOGW(TAG,
             "GT911 init failed at 0x%02X (%s), retrying 0x%02X",
             ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS,
             esp_err_to_name(err),
             ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP);

    err = bsp_try_init_touch_with_addr(i2c_bus, ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP, touch_handle);
    if (err == ESP_OK) {
        return ESP_OK;
    }

    ESP_LOGE(TAG,
             "GT911 init failed at both addresses 0x%02X and 0x%02X",
             ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS,
             ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP);
    return err;
}

static void gba_set_status_text_locked(const char *text, lv_color_t color)
{
    if (s_status_label == NULL) {
        return;
    }

    lv_label_set_text(s_status_label, text);
    lv_obj_set_style_text_color(s_status_label, color, LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void gba_set_status_text_threadsafe(const char *text, lv_color_t color)
{
    if (lvgl_port_lock(200)) {
        gba_set_status_text_locked(text, color);
        lvgl_port_unlock();
    }
}

static void gba_loading_anim_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if ((!s_gba_loading) || (s_status_label == NULL)) {
        return;
    }

    static const char *loading_text[] = {
        "Loading ROM.",
        "Loading ROM..",
        "Loading ROM...",
    };

    gba_set_status_text_locked(loading_text[s_loading_anim_phase], lv_color_hex(0xFFD18A));
    s_loading_anim_phase = (uint8_t)((s_loading_anim_phase + 1U) % 3U);
}

static int16_t gba_input_state_cb(unsigned port, unsigned device, unsigned index, unsigned id)
{
    (void)index;
    if ((port != 0U) || (device != RETRO_DEVICE_JOYPAD)) {
        return 0;
    }

    uint32_t mask = 0;
    uint32_t pressed = s_gp_pressed_mask;

    if (pressed & BUTTON_DOWN) {
        mask |= 1U << RETRO_DEVICE_ID_JOYPAD_DOWN;
    }
    if (pressed & BUTTON_UP) {
        mask |= 1U << RETRO_DEVICE_ID_JOYPAD_UP;
    }
    if (pressed & BUTTON_LEFT) {
        mask |= 1U << RETRO_DEVICE_ID_JOYPAD_LEFT;
    }
    if (pressed & BUTTON_RIGHT) {
        mask |= 1U << RETRO_DEVICE_ID_JOYPAD_RIGHT;
    }
    if (pressed & BUTTON_START) {
        mask |= 1U << RETRO_DEVICE_ID_JOYPAD_START;
    }
    if (pressed & BUTTON_SELECT) {
        mask |= 1U << RETRO_DEVICE_ID_JOYPAD_SELECT;
    }
    if (pressed & BUTTON_B) {
        mask |= 1U << RETRO_DEVICE_ID_JOYPAD_B;
    }
    if (pressed & BUTTON_A) {
        mask |= 1U << RETRO_DEVICE_ID_JOYPAD_A;
    }
    if (pressed & BUTTON_L) {
        mask |= 1U << RETRO_DEVICE_ID_JOYPAD_L;
    }
    if (pressed & BUTTON_R) {
        mask |= 1U << RETRO_DEVICE_ID_JOYPAD_R;
    }

    if (id == RETRO_DEVICE_ID_JOYPAD_MASK) {
        return (int16_t)mask;
    }
    return (mask & (1U << id)) ? 1 : 0;
}

static esp_err_t gba_decompress_rom_to_psram(void)
{
    const uint8_t *blob = pokemon_gba_zlib_start;
    size_t blob_size = (size_t)(pokemon_gba_zlib_end - pokemon_gba_zlib_start);

    if (blob_size <= sizeof(uint32_t)) {
        ESP_LOGE(TAG, "Embedded ROM blob is invalid");
        return ESP_FAIL;
    }

    uint32_t expected_size = ((uint32_t)blob[0])
                           | ((uint32_t)blob[1] << 8)
                           | ((uint32_t)blob[2] << 16)
                           | ((uint32_t)blob[3] << 24);
    const uint8_t *compressed = blob + sizeof(uint32_t);
    size_t compressed_size = blob_size - sizeof(uint32_t);

    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t largest_psram = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG,
             "embedded pokemon.gba.zlib payload=%u bytes, expected_rom=%u bytes, free_psram=%u bytes, largest_psram_block=%u bytes",
             (unsigned int)compressed_size,
             (unsigned int)expected_size,
             (unsigned int)free_psram,
             (unsigned int)largest_psram);

    if ((expected_size == 0U) || (expected_size > GBA_ROM_MAX_BYTES)) {
        ESP_LOGE(TAG, "ROM size invalid: %u", (unsigned int)expected_size);
        return ESP_FAIL;
    }

    if (s_rom_buffer != NULL) {
        heap_caps_free(s_rom_buffer);
        s_rom_buffer = NULL;
        s_rom_size = 0;
    }

    uint8_t *rom = heap_caps_malloc(expected_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (rom == NULL) {
        ESP_LOGE(TAG, "Failed to allocate ROM buffer in PSRAM");
        return ESP_ERR_NO_MEM;
    }

    size_t decompressed_size = tinfl_decompress_mem_to_mem(rom,
                                                           expected_size,
                                                           compressed,
                                                           compressed_size,
                                                           TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
    if ((decompressed_size == TINFL_DECOMPRESS_MEM_TO_MEM_FAILED) || (decompressed_size != expected_size)) {
        ESP_LOGE(TAG,
                 "ROM decompress failed (produced=%u expected=%u)",
                 (unsigned int)decompressed_size,
                 (unsigned int)expected_size);
        heap_caps_free(rom);
        return ESP_FAIL;
    }

    if (!esp_ptr_external_ram(rom)) {
        ESP_LOGW(TAG, "ROM buffer is not in PSRAM");
    }

    s_rom_buffer = rom;
    s_rom_size = expected_size;
    ESP_LOGI(TAG,
             "ROM decompressed size=%u bytes, storage=%s",
             (unsigned int)expected_size,
             esp_ptr_external_ram(rom) ? "PSRAM" : "internal RAM");
    return ESP_OK;
}

static void gba_draw_wait_pattern(uint32_t phase)
{
    if (s_game_canvas_buf == NULL) {
        return;
    }

    uint32_t bar_x = phase % GBA_CANVAS_WIDTH;
    for (uint32_t y = 0; y < GBA_CANVAS_HEIGHT; y++) {
        for (uint32_t x = 0; x < GBA_CANVAS_WIDTH; x++) {
            bool checker = (((x / 16U) + (y / 16U)) & 1U) != 0;
            uint16_t color = checker ? 0x3186 : 0x2104;
            if ((x == bar_x) || (x + 1U == bar_x)) {
                color = 0xFD20;
            }
            s_game_canvas_buf[y * GBA_CANVAS_WIDTH + x] = color;
        }
    }
}

static bool gba_copy_frame_to_canvas(void)
{
    if ((s_gp_frame_buf == NULL) || (s_game_canvas_buf == NULL)) {
        return false;
    }

    bool has_non_zero = gba_source_frame_has_non_zero();

    if (gba_scale_frame_with_ppa()) {
        return has_non_zero;
    }

    gba_prepare_scale_lut();
    for (uint32_t y = 0; y < GBA_CANVAS_HEIGHT; y++) {
        const uint16_t *src_row = s_gp_frame_buf + ((uint32_t)s_scale_y_lut[y] * GBA_FRAME_WIDTH);
        uint16_t *dst_row = s_game_canvas_buf + ((uint32_t)y * GBA_CANVAS_WIDTH);

        for (uint32_t x = 0; x < GBA_CANVAS_WIDTH; x++) {
            dst_row[x] = src_row[s_scale_x_lut[x]];
        }
    }

    return has_non_zero;
}

static esp_err_t gba_init_runtime(void)
{
    if (gba_ensure_canvas_buffer() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate LVGL canvas buffer");
        return ESP_ERR_NO_MEM;
    }

    if (s_gp_frame_buf == NULL) {
        s_gp_frame_buf = heap_caps_malloc(GBA_SCREEN_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_gp_frame_buf == NULL) {
            s_gp_frame_buf = heap_caps_malloc(GBA_SCREEN_BUFFER_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
    }
    if (s_gp_frame_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate gpSP frame buffer");
        return ESP_ERR_NO_MEM;
    }

    memset(s_game_canvas_buf, 0, GBA_CANVAS_BYTES);
    memset(s_gp_frame_buf, 0, GBA_SCREEN_BUFFER_SIZE);
    gba_screen_pixels = s_gp_frame_buf;
    gba_prepare_scale_lut();

    if (!s_ppa_srm_ready) {
        esp_err_t ppa_err = gba_init_ppa_client();
        if ((ppa_err != ESP_OK) && !s_ppa_srm_warned) {
            ESP_LOGW(TAG, "PPA init failed (%s), using CPU scaler", esp_err_to_name(ppa_err));
            s_ppa_srm_warned = true;
        }
    }

    if (!s_gp_core_initialized) {
        init_gamepak_buffer();
        init_sound();
        s_gp_core_initialized = true;
    }

    esp_err_t rom_err = gba_decompress_rom_to_psram();
    if (rom_err != ESP_OK) {
        return rom_err;
    }

    size_t bios_size = (size_t)(open_gba_bios_bin_end - open_gba_bios_bin_start);
    if (bios_size != (1024U * 16U)) {
        ESP_LOGE(TAG, "open_gba_bios.bin has invalid size: %u", (unsigned int)bios_size);
        return ESP_FAIL;
    }

    memcpy(bios_rom, open_gba_bios_bin_start, 1024U * 16U);
    memset(gamepak_backup, 0xFF, sizeof(gamepak_backup));

    libretro_supports_bitmasks = true;
    retro_set_input_state(gba_input_state_cb);

    set_gamepak_source_memory(s_rom_buffer, (u32)s_rom_size);
    if (load_gamepak(NULL,
                     "embedded://pokemon.gba",
                     FEAT_DISABLE,
                     FEAT_DISABLE,
                     SERIAL_MODE_DISABLED) != 0) {
        ESP_LOGE(TAG, "load_gamepak failed");
        return ESP_FAIL;
    }

    reset_gba();
    s_gp_pressed_mask = 0;
    s_gba_frame_ready = false;
    s_last_frame_counter = frame_counter;
    return ESP_OK;
}

static void gba_loader_task(void *arg)
{
    (void)arg;

    s_gba_init_result = gba_init_runtime();
    if (s_gba_init_result == ESP_OK) {
        s_gba_running = true;
        gba_set_status_text_threadsafe("ROM loaded. Emulator running.", lv_color_hex(0x8FE39D));
        ESP_LOGI(TAG, "GBA emulator rendering started");
    } else {
        s_gba_running = false;
        gba_set_status_text_threadsafe("ROM load failed - tap START again", lv_color_hex(0xFFD18A));
        ESP_LOGW(TAG, "GBA runtime init failed: %s", esp_err_to_name(s_gba_init_result));
    }

    s_gba_loading = false;
    vTaskDelete(NULL);
}

static void gba_request_start_loading(void)
{
    if (s_gba_running) {
        gba_set_status_text_locked("Emulator already running", lv_color_hex(0x8FE39D));
        return;
    }

    if (s_gba_loading) {
        return;
    }

    s_gba_loading = true;
    s_loading_anim_phase = 0;
    s_frame_stream_status_deadline = 0;
    gba_set_status_text_locked("Loading ROM.", lv_color_hex(0xFFD18A));

    BaseType_t ok = xTaskCreate(gba_loader_task, "gba_loader", 24576, NULL, 5, NULL);
    if (ok != pdPASS) {
        s_gba_loading = false;
        gba_set_status_text_locked("Failed to create loader task", lv_color_hex(0xFFD18A));
        ESP_LOGE(TAG, "Failed to create ROM loader task");
    }
}

static void gba_power_off_game_locked(void)
{
    if (s_gba_loading) {
        gba_set_status_text_locked("Power off unavailable while loading", lv_color_hex(0xFFD18A));
        return;
    }

    if (!s_gba_running) {
        gba_set_status_text_locked("Game already powered off", lv_color_hex(0xFFD18A));
        return;
    }

    s_gp_pressed_mask = 0;
    s_gba_running = false;
    s_gba_frame_ready = false;
    s_frame_stream_status_deadline = 0;

    if (s_game_canvas != NULL) {
        lv_canvas_fill_bg(s_game_canvas, lv_color_hex(0x000000), LV_OPA_COVER);
        lv_obj_invalidate(s_game_canvas);
    }

    gba_set_status_text_locked("Game powered off - tap START", lv_color_hex(0xFFD18A));
}

static void gba_button_event_cb(lv_event_t *event)
{
    const gba_button_t *button = (const gba_button_t *)lv_event_get_user_data(event);
    if (button == NULL) {
        return;
    }

    lv_event_code_t code = lv_event_get_code(event);
    if ((button == &BTN_START) && (code == LV_EVENT_CLICKED) && !s_gba_running) {
        gba_request_start_loading();
        return;
    }

    if ((button == &BTN_POWER) && (code == LV_EVENT_CLICKED)) {
        gba_power_off_game_locked();
        return;
    }

    if (code == LV_EVENT_PRESSED) {
        s_gp_pressed_mask |= button->button_mask;
    } else if ((code == LV_EVENT_RELEASED) || (code == LV_EVENT_PRESS_LOST)) {
        s_gp_pressed_mask &= ~button->button_mask;
    }
}

static void gba_disable_scroll(lv_obj_t *obj)
{
    if (obj == NULL) {
        return;
    }

    lv_obj_clear_flag(obj,
                      LV_OBJ_FLAG_SCROLLABLE
                          | LV_OBJ_FLAG_SCROLL_CHAIN_HOR
                          | LV_OBJ_FLAG_SCROLL_CHAIN_VER
                          | LV_OBJ_FLAG_SCROLL_ELASTIC
                          | LV_OBJ_FLAG_SCROLL_MOMENTUM);
    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(obj, LV_DIR_NONE);
}

static lv_obj_t *create_gba_button(lv_obj_t *parent,
                                   const gba_button_t *button,
                                   lv_coord_t width,
                                   lv_coord_t height,
                                   bool circular,
                                   lv_color_t idle_color,
                                   lv_color_t pressed_color)
{
    lv_obj_t *obj = lv_button_create(parent);
    lv_obj_set_size(obj, width, height);
    lv_obj_set_style_radius(obj, circular ? LV_RADIUS_CIRCLE : 12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(obj, idle_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(obj, pressed_color, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(obj, lv_color_hex(0x1B1B1B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(obj, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(obj, LV_OPA_30, LV_PART_MAIN | LV_STATE_DEFAULT);
    gba_disable_scroll(obj);

    lv_obj_t *label = lv_label_create(obj);
    lv_label_set_text(label, button->label);
    lv_obj_set_style_text_color(label, lv_color_hex(0xF3F3F3), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(label);
    lv_obj_clear_flag(label, LV_OBJ_FLAG_CLICKABLE);
    gba_disable_scroll(label);

    lv_obj_add_event_cb(obj, gba_button_event_cb, LV_EVENT_PRESSED, (void *)button);
    lv_obj_add_event_cb(obj, gba_button_event_cb, LV_EVENT_RELEASED, (void *)button);
    lv_obj_add_event_cb(obj, gba_button_event_cb, LV_EVENT_PRESS_LOST, (void *)button);
    if ((button == &BTN_START) || (button == &BTN_POWER)) {
        lv_obj_add_event_cb(obj, gba_button_event_cb, LV_EVENT_CLICKED, (void *)button);
    }
    return obj;
}

static void create_gba_ui(void)
{
    lv_obj_t *screen = lv_screen_active();
    gba_disable_scroll(screen);
    lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x5F7297), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(screen, lv_color_hex(0x182136), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *bg_wallpaper = lv_obj_create(screen);
    lv_obj_remove_style_all(bg_wallpaper);
    lv_obj_set_size(bg_wallpaper, LCD_H_RES, LCD_V_RES);
    lv_obj_center(bg_wallpaper);
    lv_obj_set_style_radius(bg_wallpaper, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(bg_wallpaper, lv_color_hex(0x7689AD), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(bg_wallpaper, lv_color_hex(0x1D2942), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(bg_wallpaper, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(bg_wallpaper, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(bg_wallpaper, LV_OBJ_FLAG_CLICKABLE);
    gba_disable_scroll(bg_wallpaper);

    lv_obj_t *bg_band_top = lv_obj_create(screen);
    lv_obj_remove_style_all(bg_band_top);
    lv_obj_set_size(bg_band_top, LCD_H_RES + 120, 78);
    lv_obj_align(bg_band_top, LV_ALIGN_TOP_MID, 0, 56);
    lv_obj_set_style_radius(bg_band_top, 44, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(bg_band_top, lv_color_hex(0xA3B2CD), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(bg_band_top, LV_OPA_30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(bg_band_top, LV_OBJ_FLAG_CLICKABLE);
    gba_disable_scroll(bg_band_top);

    lv_obj_t *bg_band_bottom = lv_obj_create(screen);
    lv_obj_remove_style_all(bg_band_bottom);
    lv_obj_set_size(bg_band_bottom, LCD_H_RES + 140, 96);
    lv_obj_align(bg_band_bottom, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_set_style_radius(bg_band_bottom, 48, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(bg_band_bottom, lv_color_hex(0x6F74B0), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(bg_band_bottom, LV_OPA_30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(bg_band_bottom, LV_OBJ_FLAG_CLICKABLE);
    gba_disable_scroll(bg_band_bottom);

    lv_obj_t *bg_mark = lv_label_create(screen);
    lv_label_set_text(bg_mark, "GAME BOY ADVANCE");
    lv_obj_set_style_text_color(bg_mark, lv_color_hex(0x051a47), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(bg_mark, LV_ALIGN_TOP_MID, 0, 86);
    lv_obj_clear_flag(bg_mark, LV_OBJ_FLAG_CLICKABLE);
    gba_disable_scroll(bg_mark);

    lv_obj_t *bg_arc_top = lv_obj_create(screen);
    lv_obj_remove_style_all(bg_arc_top);
    lv_obj_set_size(bg_arc_top, LCD_H_RES + 150, 240);
    lv_obj_align(bg_arc_top, LV_ALIGN_TOP_MID, 0, -90);
    lv_obj_set_style_radius(bg_arc_top, 140, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(bg_arc_top, lv_color_hex(0xBFCBE2), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(bg_arc_top, LV_OPA_30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(bg_arc_top, LV_OBJ_FLAG_CLICKABLE);
    gba_disable_scroll(bg_arc_top);

    lv_obj_t *bg_arc_bottom = lv_obj_create(screen);
    lv_obj_remove_style_all(bg_arc_bottom);
    lv_obj_set_size(bg_arc_bottom, LCD_H_RES + 120, 230);
    lv_obj_align(bg_arc_bottom, LV_ALIGN_BOTTOM_MID, 0, 86);
    lv_obj_set_style_radius(bg_arc_bottom, 140, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(bg_arc_bottom, lv_color_hex(0x6F6AA5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(bg_arc_bottom, LV_OPA_20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(bg_arc_bottom, LV_OBJ_FLAG_CLICKABLE);
    gba_disable_scroll(bg_arc_bottom);

    lv_obj_t *bg_strip = lv_obj_create(screen);
    lv_obj_remove_style_all(bg_strip);
    lv_obj_set_size(bg_strip, LCD_H_RES + 80, 44);
    lv_obj_align(bg_strip, LV_ALIGN_CENTER, 0, -4);
    lv_obj_set_style_radius(bg_strip, 24, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(bg_strip, lv_color_hex(0x9DAED0), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(bg_strip, LV_OPA_20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(bg_strip, LV_OBJ_FLAG_CLICKABLE);
    gba_disable_scroll(bg_strip);

    lv_obj_t *viewport = lv_obj_create(screen);
    lv_obj_remove_style_all(viewport);
    lv_obj_set_size(viewport, GBA_VIEWPORT_WIDTH, GBA_VIEWPORT_HEIGHT);
    lv_obj_align(viewport, LV_ALIGN_CENTER, 0, -50);
    lv_obj_set_style_radius(viewport, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(viewport, GBA_VIEWPORT_BORDER_WIDTH, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(viewport, lv_color_hex(0x81899A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(viewport, lv_color_hex(0x050607), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(viewport, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(viewport, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    gba_disable_scroll(viewport);

    if (gba_ensure_canvas_buffer() == ESP_OK) {
        s_game_canvas = lv_canvas_create(viewport);
        lv_canvas_set_buffer(s_game_canvas,
                             s_game_canvas_buf,
                             GBA_CANVAS_WIDTH,
                             GBA_CANVAS_HEIGHT,
                             LV_COLOR_FORMAT_RGB565);
        lv_obj_set_size(s_game_canvas, GBA_CANVAS_WIDTH, GBA_CANVAS_HEIGHT);
        lv_image_set_inner_align(s_game_canvas, LV_IMAGE_ALIGN_TOP_LEFT);
        lv_obj_align(s_game_canvas, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_canvas_fill_bg(s_game_canvas, lv_color_hex(0x000000), LV_OPA_COVER);
        lv_obj_clear_flag(s_game_canvas, LV_OBJ_FLAG_CLICKABLE);
        gba_disable_scroll(s_game_canvas);
    } else {
        lv_obj_t *error = lv_label_create(viewport);
        lv_label_set_text(error, "GBA frame buffer unavailable");
        lv_obj_set_style_text_color(error, lv_color_hex(0xF6C56B), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_center(error);
        lv_obj_clear_flag(error, LV_OBJ_FLAG_CLICKABLE);
        gba_disable_scroll(error);
    }

    lv_obj_t *btn_l = create_gba_button(screen,
                                        &BTN_L,
                                        120,
                                        42,
                                        false,
                                        lv_color_hex(0x4D4F5B),
                                        lv_color_hex(0x686B7C));
    lv_obj_align_to(btn_l, viewport, LV_ALIGN_OUT_TOP_LEFT, 8, -18);

    lv_obj_t *btn_r = create_gba_button(screen,
                                        &BTN_R,
                                        120,
                                        42,
                                        false,
                                        lv_color_hex(0x4D4F5B),
                                        lv_color_hex(0x686B7C));
    lv_obj_align_to(btn_r, viewport, LV_ALIGN_OUT_TOP_RIGHT, -8, -18);

    lv_obj_t *btn_power = create_gba_button(screen,
                                            &BTN_POWER,
                                            112,
                                            36,
                                            false,
                                            lv_color_hex(0x7B3F40),
                                            lv_color_hex(0x9B5152));
    lv_obj_align_to(btn_power, viewport, LV_ALIGN_OUT_TOP_MID, 0, -14);

    lv_obj_t *dpad = lv_obj_create(screen);
    lv_obj_remove_style_all(dpad);
    lv_obj_set_size(dpad, 186, 186);
    lv_obj_align_to(dpad, viewport, LV_ALIGN_OUT_BOTTOM_LEFT, 8, 62);
    gba_disable_scroll(dpad);

    lv_obj_t *up = create_gba_button(dpad, &BTN_UP, 46, 46, true, lv_color_hex(0x3E424A), lv_color_hex(0x545A64));
    lv_obj_align(up, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t *down = create_gba_button(dpad, &BTN_DOWN, 46, 46, true, lv_color_hex(0x3E424A), lv_color_hex(0x545A64));
    lv_obj_align(down, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_t *left = create_gba_button(dpad, &BTN_LEFT, 46, 46, true, lv_color_hex(0x3E424A), lv_color_hex(0x545A64));
    lv_obj_align(left, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *right = create_gba_button(dpad, &BTN_RIGHT, 46, 46, true, lv_color_hex(0x3E424A), lv_color_hex(0x545A64));
    lv_obj_align(right, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_t *actions = lv_obj_create(screen);
    lv_obj_remove_style_all(actions);
    lv_obj_set_size(actions, 164, 120);
    lv_obj_align_to(actions, viewport, LV_ALIGN_OUT_BOTTOM_RIGHT, -8, 38);
    gba_disable_scroll(actions);

    lv_obj_t *btn_a = create_gba_button(actions, &BTN_A, 64, 64, true, lv_color_hex(0xB33F3F), lv_color_hex(0xCB5959));
    lv_obj_align(btn_a, LV_ALIGN_RIGHT_MID, 0, -8);

    lv_obj_t *btn_b = create_gba_button(actions, &BTN_B, 64, 64, true, lv_color_hex(0x74438D), lv_color_hex(0x8E5AA9));
    lv_obj_align(btn_b, LV_ALIGN_LEFT_MID, 0, 8);

    lv_obj_t *btn_select = create_gba_button(screen,
                                             &BTN_SELECT,
                                             118,
                                             40,
                                             false,
                                             lv_color_hex(0x474C57),
                                             lv_color_hex(0x5D6574));
    lv_obj_align_to(btn_select, viewport, LV_ALIGN_OUT_BOTTOM_MID, -74, 20);

    lv_obj_t *btn_start = create_gba_button(screen,
                                            &BTN_START,
                                            118,
                                            40,
                                            false,
                                            lv_color_hex(0x474C57),
                                            lv_color_hex(0x5D6574));
    lv_obj_align_to(btn_start, viewport, LV_ALIGN_OUT_BOTTOM_MID, 74, 20);

    s_status_label = lv_label_create(viewport);
    lv_obj_align(s_status_label, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_clear_flag(s_status_label, LV_OBJ_FLAG_CLICKABLE);
    gba_disable_scroll(s_status_label);

    if (s_gba_running) {
        gba_set_status_text_locked("ROM loaded. Emulator running.", lv_color_hex(0x8FE39D));
    } else if (s_gba_loading) {
        gba_set_status_text_locked("Loading ROM.", lv_color_hex(0xFFD18A));
    } else {
        gba_set_status_text_locked("Tap START to load ROM", lv_color_hex(0xFFD18A));
    }

    if (!s_layout_debug_logged) {
        int32_t v_w = lv_obj_get_width(viewport);
        int32_t v_h = lv_obj_get_height(viewport);
        int32_t vc_w = lv_obj_get_content_width(viewport);
        int32_t vc_h = lv_obj_get_content_height(viewport);
        int32_t c_w = (s_game_canvas != NULL) ? lv_obj_get_width(s_game_canvas) : -1;
        int32_t c_h = (s_game_canvas != NULL) ? lv_obj_get_height(s_game_canvas) : -1;
        int32_t c_x = (s_game_canvas != NULL) ? lv_obj_get_x(s_game_canvas) : -1;
        int32_t c_y = (s_game_canvas != NULL) ? lv_obj_get_y(s_game_canvas) : -1;
        int32_t p_l = lv_obj_get_style_pad_left(viewport, LV_PART_MAIN);
        int32_t p_r = lv_obj_get_style_pad_right(viewport, LV_PART_MAIN);
        int32_t p_t = lv_obj_get_style_pad_top(viewport, LV_PART_MAIN);
        int32_t p_b = lv_obj_get_style_pad_bottom(viewport, LV_PART_MAIN);

        ESP_LOGI(TAG,
                 "Layout dbg viewport=%ldx%ld content=%ldx%ld canvas=%ldx%ld pos=(%ld,%ld) pad=(%ld,%ld,%ld,%ld)",
                 (long)v_w,
                 (long)v_h,
                 (long)vc_w,
                 (long)vc_h,
                 (long)c_w,
                 (long)c_h,
                 (long)c_x,
                 (long)c_y,
                 (long)p_l,
                 (long)p_r,
                 (long)p_t,
                 (long)p_b);
        s_layout_debug_logged = true;
    }

    if (s_loading_timer == NULL) {
        s_loading_timer = lv_timer_create(gba_loading_anim_timer_cb, 260, NULL);
    }
}

static void gba_emulation_task(void *arg)
{
    (void)arg;
    uint32_t wait_phase = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        if (s_gba_running && (s_gp_frame_buf != NULL) && (s_game_canvas != NULL) && (s_game_canvas_buf != NULL)) {
            uint32_t frame_before = frame_counter;
            TickType_t now_tick = xTaskGetTickCount();

            update_input();
            rumble_frame_reset();
            clear_gamepak_stickybits();
            execute_arm(execute_cycles);

            bool frame_advanced = (frame_counter != frame_before);
            if (frame_advanced) {
                s_last_frame_counter = frame_counter;
            }

            if (lvgl_port_lock(20)) {
                bool should_invalidate = false;
                bool has_frame = false;
                if (frame_advanced) {
                    has_frame = gba_copy_frame_to_canvas();
                    should_invalidate = has_frame;
                }

                if (has_frame && !s_gba_frame_ready) {
                    s_gba_frame_ready = true;
                    gba_set_status_text_locked("Frame stream active", lv_color_hex(0x8FE39D));
                    s_frame_stream_status_deadline = now_tick + pdMS_TO_TICKS(GBA_FRAME_STREAM_STATUS_MS);
                }

                if (s_gba_frame_ready && (s_frame_stream_status_deadline != 0U)
                    && gba_tick_reached(now_tick, s_frame_stream_status_deadline)) {
                    if ((s_status_label != NULL)
                        && (strcmp(lv_label_get_text(s_status_label), "Frame stream active") == 0)) {
                        gba_set_status_text_locked("", lv_color_hex(0x8FE39D));
                    }
                    s_frame_stream_status_deadline = 0;
                }

                if (!s_gba_frame_ready) {
                    gba_draw_wait_pattern(wait_phase++);
                    should_invalidate = true;
                }

                if (should_invalidate) {
                    lv_obj_invalidate(s_game_canvas);
                }
                lvgl_port_unlock();
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(GBA_TARGET_FRAME_TIME_MS));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(bsp_enable_dsi_phy_power());
    ESP_ERROR_CHECK(bsp_init_backlight());

    esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_config = ST7701_PANEL_BUS_DSI_2CH_CONFIG();
    bus_config.lane_bit_rate_mbps = LCD_MIPI_DSI_LANE_BITRATE_MBPS;
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));

    esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
    esp_lcd_dbi_io_config_t dbi_config = ST7701_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io));

    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 34,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .num_fbs = 2,
        .video_timing = {
            .h_size = LCD_H_RES,
            .v_size = LCD_V_RES,
            .hsync_pulse_width = 12,
            .hsync_back_porch = 42,
            .hsync_front_porch = 42,
            .vsync_pulse_width = 2,
            .vsync_back_porch = 8,
            .vsync_front_porch = 166,
        },
        .flags = {
            .use_dma2d = true,
        },
    };

    st7701_vendor_config_t vendor_config = {
        .init_cmds = lcd_cmd,
        .init_cmds_size = sizeof(lcd_cmd) / sizeof(st7701_lcd_init_cmd_t),
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
        .flags = {
            .use_mipi_interface = 1,
        },
    };

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };

    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7701(mipi_dbi_io, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    bsp_set_backlight(true);

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = mipi_dbi_io,
        .panel_handle = panel_handle,
        .control_handle = NULL,
        .buffer_size = LCD_H_RES * 50,
        .double_buffer = false,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
            .sw_rotate = false,
            .full_refresh = false,
            .direct_mode = false,
        },
    };
    const lvgl_port_display_dsi_cfg_t dsi_disp_cfg = {
        .flags = {
            .avoid_tearing = false,
        },
    };

    lv_display_t *display = lvgl_port_add_disp_dsi(&disp_cfg, &dsi_disp_cfg);
    ESP_ERROR_CHECK(display ? ESP_OK : ESP_FAIL);

    esp_lcd_touch_handle_t touch_handle = NULL;
    ESP_ERROR_CHECK(bsp_init_touch(&touch_handle));

    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = display,
        .handle = touch_handle,
        .scale = {
            .x = 1.0f,
            .y = 1.0f,
        },
    };

    lv_indev_t *touch_input = lvgl_port_add_touch(&touch_cfg);
    ESP_ERROR_CHECK(touch_input ? ESP_OK : ESP_FAIL);

    s_gba_init_result = ESP_ERR_INVALID_STATE;
    s_gba_loading = false;
    s_gba_running = false;
    s_gba_frame_ready = false;
    s_loading_anim_phase = 0;
    s_gp_pressed_mask = 0;
    s_emu_task_started = false;
    s_gp_core_initialized = false;

    if (lvgl_port_lock(0)) {
        create_gba_ui();
        lvgl_port_unlock();
    }

    if (!s_emu_task_started) {
        if (xTaskCreate(gba_emulation_task, "gba_emu", 18432, NULL, 4, NULL) == pdPASS) {
            s_emu_task_started = true;
        } else {
            ESP_LOGE(TAG, "Failed to create emulator task");
        }
    }

    ESP_LOGI(TAG, "Tap START to load ROM and begin emulation");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
