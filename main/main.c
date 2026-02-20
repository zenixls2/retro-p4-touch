#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ppa.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7701.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_lvgl_port.h"
#include "esp_lvgl_port_touch.h"
#include "esp_sntp.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "lvgl.h"
#include "miniz.h"
#include "nvs_flash.h"
#include "driver/sdmmc_default_configs.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"

#if __has_include("esp_hosted.h")
#include "esp_hosted.h"
#define GBA_HAS_ESP_HOSTED 1
#else
#define GBA_HAS_ESP_HOSTED 0
#endif

#if defined(CONFIG_BT_ENABLED) && CONFIG_BT_ENABLED
#include "esp_bt_main.h"
#include "esp_bluedroid_hci.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "esp_hosted_bluedroid.h"
#include "esp_hosted_misc.h"
#define GBA_BT_STACK_AVAILABLE 1
#else
#define GBA_BT_STACK_AVAILABLE 0
#endif

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
#define GBA_VIEWPORT_WIDTH 480
#define GBA_VIEWPORT_HEIGHT 320
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
#define GBA_INFO_BAR_HEIGHT 42
#define GBA_INFO_BAR_PAD_X 12
#define GBA_WIFI_AP_LIST_MAX 20
#define GBA_WIFI_SSID_LEN 32
#define GBA_WIFI_PASS_LEN 64
#define GBA_BT_SCAN_DEVICE_MAX 20
#define GBA_BT_NAME_MAX_LEN 33
#define GBA_BT_SCAN_DURATION_SEC 6
#define GBA_BT_NAME_PROBE_APP_ID 0x32
#define GBA_BT_NAME_PROBE_MAX_PER_SCAN 2
#define GBA_BT_NAME_PROBE_MIN_RSSI -70
#define GBA_BT_NAME_PROBE_MIN_RSSI_RELAXED -90
#define GBA_BT_NAME_PROBE_HISTORY_SIZE 16
#define GBA_BT_NAME_PROBE_COOLDOWN_BASE_MS 30000
#define GBA_BT_NAME_PROBE_COOLDOWN_STEP_MS 15000
#define GBA_BT_NAME_PROBE_COOLDOWN_MAX_MS 120000
#define GBA_TZ_LOOKUP_URL "http://worldtimeapi.org/api/ip.txt"
#define GBA_TZ_LOOKUP_FALLBACK_URL "http://ip-api.com/line/?fields=offset"
#define GBA_TZ_HTTP_BUF_SIZE 512
#define GBA_SD_MOUNT_POINT "/sdcard"
#define GBA_SD_ROM_PATH GBA_SD_MOUNT_POINT "/pokemon.gba"
#define GBA_SD_SAVE_PATH GBA_SD_MOUNT_POINT "/pokemon.sav"
#define GBA_SD_SAVE_TMP_PATH GBA_SD_MOUNT_POINT "/pokemon.sav.tmp"
#define GBA_SD_SAVE_FLUSH_MS 5000
#define GBA_SD_SAVE_POLL_MS 250
#define GBA_SD_MOUNT_RETRY_COUNT 3
#define GBA_SD_MOUNT_RETRY_DELAY_MS 250
#define GBA_SD_MOUNT_COOLDOWN_MS 5000

typedef struct {
    const char *label;
    uint32_t button_mask;
} gba_button_t;

typedef struct {
    char ssid[GBA_WIFI_SSID_LEN + 1];
    int8_t rssi;
    wifi_auth_mode_t authmode;
} gba_wifi_ap_t;

typedef enum {
    GBA_BT_NAME_SOURCE_NONE = 0,
    GBA_BT_NAME_SOURCE_HEURISTIC = 1,
    GBA_BT_NAME_SOURCE_SHORT = 2,
    GBA_BT_NAME_SOURCE_COMPLETE = 3,
} gba_bt_name_source_t;

typedef struct {
    uint8_t bda[6];
    char name[GBA_BT_NAME_MAX_LEN];
    int8_t rssi;
    gba_bt_name_source_t name_source;
    uint8_t addr_type;
    bool connectable;
    bool name_probe_attempted;
    bool has_mfg_company_id;
    uint16_t mfg_company_id;
    bool has_service_uuid16;
    uint16_t service_uuid16;
    bool has_service_data_uuid16;
    uint16_t service_data_uuid16;
    bool has_tx_power;
    int8_t tx_power_dbm;
    bool has_appearance;
    uint16_t appearance;
    bool has_flags;
    uint8_t adv_flags;
    bool has_scan_rsp;
} gba_bt_device_t;

typedef struct {
    bool used;
    uint8_t bda[6];
    uint8_t fail_count;
    TickType_t cooldown_until;
} gba_bt_probe_history_t;

typedef enum {
    GBA_INFO_ITEM_WIFI = 1,
    GBA_INFO_ITEM_BLUETOOTH = 2,
} gba_info_item_t;

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
static lv_timer_t *s_info_timer;

static lv_obj_t *s_wifi_icon_btn;
static lv_obj_t *s_wifi_icon_label;
static lv_obj_t *s_bt_icon_btn;
static lv_obj_t *s_bt_icon_label;
static lv_obj_t *s_info_time_label;

static lv_obj_t *s_wifi_popup;
static lv_obj_t *s_wifi_popup_status_label;
static lv_obj_t *s_wifi_ap_list;
static lv_obj_t *s_wifi_password_popup;
static lv_obj_t *s_wifi_password_ta;
static lv_obj_t *s_wifi_password_keyboard;

static lv_obj_t *s_bt_popup;
static lv_obj_t *s_bt_popup_status_label;
static lv_obj_t *s_bt_popup_scan_btn_label;
static lv_obj_t *s_bt_device_list;

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

static SemaphoreHandle_t s_wifi_lock;
static SemaphoreHandle_t s_bt_lock;
static bool s_wifi_ready;
static bool s_wifi_connecting;
static bool s_wifi_connected;
static bool s_wifi_scanning;
static bool s_wifi_scan_pending;
static bool s_wifi_list_dirty;
static bool s_wifi_state_dirty;
static char s_wifi_connected_ssid[GBA_WIFI_SSID_LEN + 1];
static char s_wifi_selected_ssid[GBA_WIFI_SSID_LEN + 1];
static char s_wifi_country_code[4];
static gba_wifi_ap_t s_wifi_ap_list_cache[GBA_WIFI_AP_LIST_MAX];
static size_t s_wifi_ap_count;
static bool s_bt_ready;
static bool s_bt_scanning;
static bool s_bt_list_dirty;
static bool s_bt_state_dirty;
static gba_bt_device_t s_bt_devices[GBA_BT_SCAN_DEVICE_MAX];
static size_t s_bt_device_count;
#if GBA_BT_STACK_AVAILABLE
static esp_gatt_if_t s_bt_gattc_if = ESP_GATT_IF_NONE;
static bool s_bt_gattc_ready;
static bool s_bt_name_probe_busy;
static bool s_bt_name_probe_succeeded;
static bool s_bt_name_probe_bda_valid;
static uint8_t s_bt_name_probe_bda[6];
static int s_bt_name_probe_index = -1;
static uint16_t s_bt_name_probe_conn_id;
static bool s_bt_name_probe_svc_found;
static uint16_t s_bt_name_probe_svc_start_handle;
static uint16_t s_bt_name_probe_svc_end_handle;
static uint8_t s_bt_name_probe_budget;
static gba_bt_probe_history_t s_bt_probe_history[GBA_BT_NAME_PROBE_HISTORY_SIZE];
#endif
static bool s_ntp_started;
static bool s_ntp_synced;
static bool s_timezone_fetching;
static bool s_sd_card_mounted;
static sdmmc_card_t *s_sd_card;
static sd_pwr_ctrl_handle_t s_sd_pwr_ctrl_handle;
static TickType_t s_sd_mount_retry_after;
static uint32_t s_backup_checksum;
static uint32_t s_backup_observed_checksum;
static TickType_t s_backup_flush_deadline;
static TickType_t s_backup_poll_deadline;

static esp_event_handler_instance_t s_wifi_event_instance;
static esp_event_handler_instance_t s_ip_event_instance;

static void gba_prepare_scale_lut(void);
static void gba_disable_scroll(lv_obj_t *obj);
static void gba_set_status_text_locked(const char *text, lv_color_t color);
static void gba_set_status_text_threadsafe(const char *text, lv_color_t color);
static void gba_request_start_loading(void);
static void gba_close_wifi_popup_locked(void);
static void gba_close_wifi_password_popup_locked(void);
static void gba_open_wifi_popup_locked(void);
static void gba_open_bt_popup_locked(void);
static void gba_rebuild_wifi_ap_list_locked(void);
static void gba_rebuild_bt_device_list_locked(void);
static void gba_refresh_info_ui_locked(void);
static void gba_update_time_label_locked(void);
static void gba_ntp_start_if_needed(void);
static void gba_ntp_time_sync_cb(struct timeval *tv);
static void gba_start_timezone_lookup_if_needed(void);
static esp_err_t gba_bt_init(void);
static esp_err_t gba_bt_start_scan(void);

#if GBA_BT_STACK_AVAILABLE
static void gba_bt_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gba_bt_gattc_event_handler(esp_gattc_cb_event_t event,
                                       esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param);
static void gba_bt_try_kick_name_probe(void);
static const char *gba_bt_addr_type_label(uint8_t addr_type);

static const esp_ble_scan_params_t s_bt_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
};

static const esp_bluedroid_hci_driver_operations_t s_bt_hci_ops = {
    .send = hosted_hci_bluedroid_send,
    .check_send_available = hosted_hci_bluedroid_check_send_available,
    .register_host_callback = hosted_hci_bluedroid_register_host_callback,
};

static const esp_bt_uuid_t s_bt_gap_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = 0x1800,
    },
};

static const esp_bt_uuid_t s_bt_gap_device_name_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = ESP_GATT_UUID_GAP_DEVICE_NAME,
    },
};
#endif

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

static esp_err_t gba_mount_sd_card(void)
{
    if (s_sd_card_mounted) {
        return ESP_OK;
    }

    TickType_t now = xTaskGetTickCount();
    if ((s_sd_mount_retry_after != 0U) && !gba_tick_reached(now, s_sd_mount_retry_after)) {
        return ESP_ERR_TIMEOUT;
    }

    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 64 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.pwr_ctrl_handle = NULL;

    typedef struct {
        const char *label;
        bool swap_clk_cmd;
        int width;
        int freq_khz;
    } gba_sd_mount_profile_t;

    static const gba_sd_mount_profile_t mount_profiles[] = {
        {.label = "default-4bit-defaultfreq", .swap_clk_cmd = false, .width = 4, .freq_khz = SDMMC_FREQ_DEFAULT},
        {.label = "default-1bit-defaultfreq", .swap_clk_cmd = false, .width = 1, .freq_khz = SDMMC_FREQ_DEFAULT},
        {.label = "swapped-4bit-defaultfreq", .swap_clk_cmd = true, .width = 4, .freq_khz = SDMMC_FREQ_DEFAULT},
        {.label = "swapped-1bit-defaultfreq", .swap_clk_cmd = true, .width = 1, .freq_khz = SDMMC_FREQ_DEFAULT},
        {.label = "default-1bit-probing", .swap_clk_cmd = false, .width = 1, .freq_khz = SDMMC_FREQ_PROBING},
        {.label = "swapped-1bit-probing", .swap_clk_cmd = true, .width = 1, .freq_khz = SDMMC_FREQ_PROBING},
    };

    esp_err_t err = ESP_FAIL;
    int selected_profile = -1;
    bool selected_with_ldo = false;
    sdmmc_slot_config_t slot_config = (sdmmc_slot_config_t)SDMMC_SLOT_CONFIG_DEFAULT();

    for (int ldo_pass = 0; ldo_pass < 2; ldo_pass++) {
        if (ldo_pass == 0) {
            host.pwr_ctrl_handle = NULL;
        } else {
            if (s_sd_pwr_ctrl_handle == NULL) {
                const sd_pwr_ctrl_ldo_config_t ldo_config = {
                    .ldo_chan_id = 4,
                };
                esp_err_t pwr_err = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &s_sd_pwr_ctrl_handle);
                if (pwr_err != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to configure SD LDO power control: %s", esp_err_to_name(pwr_err));
                    break;
                }
            }
            host.pwr_ctrl_handle = s_sd_pwr_ctrl_handle;
        }

        for (int profile_index = 0;
             profile_index < (int)(sizeof(mount_profiles) / sizeof(mount_profiles[0]));
             profile_index++) {
            const gba_sd_mount_profile_t *profile = &mount_profiles[profile_index];
            slot_config = (sdmmc_slot_config_t)SDMMC_SLOT_CONFIG_DEFAULT();
            slot_config.width = profile->width;
            slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
            slot_config.cd = GPIO_NUM_NC;
            slot_config.wp = GPIO_NUM_NC;
            host.max_freq_khz = profile->freq_khz;

            if (profile->swap_clk_cmd) {
                gpio_num_t temp = slot_config.clk;
                slot_config.clk = slot_config.cmd;
                slot_config.cmd = temp;
            }

            for (int attempt = 1; attempt <= GBA_SD_MOUNT_RETRY_COUNT; attempt++) {
                if (attempt > 1) {
                    vTaskDelay(pdMS_TO_TICKS(GBA_SD_MOUNT_RETRY_DELAY_MS));
                }

                err = esp_vfs_fat_sdmmc_mount(GBA_SD_MOUNT_POINT,
                                              &host,
                                              &slot_config,
                                              &mount_config,
                                              &s_sd_card);
                if (err == ESP_OK) {
                    selected_profile = profile_index;
                    selected_with_ldo = (host.pwr_ctrl_handle != NULL);
                    break;
                }

                if (err == ESP_FAIL) {
                    break;
                }
            }

            if (err == ESP_OK) {
                break;
            }
        }

        if (err == ESP_OK) {
            break;
        }

        if (err == ESP_FAIL) {
            break;
        }
    }

    if (err != ESP_OK) {
        s_sd_mount_retry_after = xTaskGetTickCount() + pdMS_TO_TICKS(GBA_SD_MOUNT_COOLDOWN_MS);
        ESP_LOGW(TAG,
                 "Failed to mount SD card at %s (slot=%d retries=%d): %s",
                 GBA_SD_MOUNT_POINT,
                 host.slot,
                 GBA_SD_MOUNT_RETRY_COUNT,
                 esp_err_to_name(err));

        if (err == ESP_FAIL) {
            ESP_LOGW(TAG, "SD card detected but FAT/exFAT mount failed; card may be unformatted or signal quality is unstable");
        }

        return err;
    }

    s_sd_mount_retry_after = 0;
    s_sd_card_mounted = true;
    ESP_LOGI(TAG,
             "SD card mounted at %s using SDMMC slot %d (%s, ldo=%s)",
             GBA_SD_MOUNT_POINT,
             host.slot,
             mount_profiles[selected_profile >= 0 ? selected_profile : 0].label,
             selected_with_ldo ? "on" : "off");
    return ESP_OK;
}

static esp_err_t gba_load_rom_from_sd_card_to_psram(void)
{
    esp_err_t err = gba_mount_sd_card();
    if (err != ESP_OK) {
        return err;
    }

    FILE *rom_file = fopen(GBA_SD_ROM_PATH, "rb");
    if (rom_file == NULL) {
        ESP_LOGW(TAG, "ROM not found on SD card: %s", GBA_SD_ROM_PATH);
        return ESP_ERR_NOT_FOUND;
    }

    if (fseek(rom_file, 0, SEEK_END) != 0) {
        fclose(rom_file);
        ESP_LOGE(TAG, "Failed to seek SD ROM file end");
        return ESP_FAIL;
    }

    long rom_size_long = ftell(rom_file);
    if (rom_size_long <= 0) {
        fclose(rom_file);
        ESP_LOGE(TAG, "Invalid SD ROM size: %ld", rom_size_long);
        return ESP_FAIL;
    }

    if ((uint32_t)rom_size_long > GBA_ROM_MAX_BYTES) {
        fclose(rom_file);
        ESP_LOGE(TAG, "SD ROM too large: %ld", rom_size_long);
        return ESP_ERR_INVALID_SIZE;
    }

    if (fseek(rom_file, 0, SEEK_SET) != 0) {
        fclose(rom_file);
        ESP_LOGE(TAG, "Failed to rewind SD ROM file");
        return ESP_FAIL;
    }

    const size_t rom_size = (size_t)rom_size_long;

    if (s_rom_buffer != NULL) {
        heap_caps_free(s_rom_buffer);
        s_rom_buffer = NULL;
        s_rom_size = 0;
    }

    uint8_t *rom = heap_caps_malloc(rom_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (rom == NULL) {
        fclose(rom_file);
        ESP_LOGE(TAG, "Failed to allocate SD ROM buffer in PSRAM");
        return ESP_ERR_NO_MEM;
    }

    size_t read_size = fread(rom, 1, rom_size, rom_file);
    fclose(rom_file);

    if (read_size != rom_size) {
        heap_caps_free(rom);
        ESP_LOGE(TAG, "Failed to read SD ROM fully (read=%u expected=%u)",
                 (unsigned int)read_size,
                 (unsigned int)rom_size);
        return ESP_FAIL;
    }

    s_rom_buffer = rom;
    s_rom_size = rom_size;

    ESP_LOGI(TAG,
             "ROM loaded from SD card: %s (%u bytes)",
             GBA_SD_ROM_PATH,
             (unsigned int)s_rom_size);
    return ESP_OK;
}

static uint32_t gba_backup_checksum32(void)
{
    const uint8_t *data = gamepak_backup;
    size_t len = sizeof(gamepak_backup);
    uint32_t hash = 2166136261u;

    for (size_t i = 0; i < len; i++) {
        hash ^= (uint32_t)data[i];
        hash *= 16777619u;
    }
    return hash;
}

static esp_err_t gba_load_backup_from_sd_card(void)
{
    esp_err_t err = gba_mount_sd_card();
    if (err != ESP_OK) {
        return err;
    }

    FILE *save_file = fopen(GBA_SD_SAVE_PATH, "rb");
    if (save_file == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    size_t read_size = fread(gamepak_backup, 1, sizeof(gamepak_backup), save_file);
    int read_error = ferror(save_file);
    fclose(save_file);

    if (read_error != 0) {
        ESP_LOGE(TAG, "Failed while reading save file: %s", GBA_SD_SAVE_PATH);
        return ESP_FAIL;
    }

    if (read_size == 0U) {
        ESP_LOGW(TAG, "Save file is empty: %s", GBA_SD_SAVE_PATH);
        return ESP_ERR_INVALID_SIZE;
    }

    if (read_size < sizeof(gamepak_backup)) {
        memset(gamepak_backup + read_size, 0xFF, sizeof(gamepak_backup) - read_size);
    }

    TickType_t now = xTaskGetTickCount();
    s_backup_checksum = gba_backup_checksum32();
    s_backup_observed_checksum = s_backup_checksum;
    s_backup_flush_deadline = 0;
    s_backup_poll_deadline = now + pdMS_TO_TICKS(GBA_SD_SAVE_POLL_MS);
    ESP_LOGI(TAG,
             "Loaded save from SD card: %s (%u/%u bytes)",
             GBA_SD_SAVE_PATH,
             (unsigned int)read_size,
             (unsigned int)sizeof(gamepak_backup));
    return ESP_OK;
}

static esp_err_t gba_flush_backup_to_sd_card(bool force)
{
    esp_err_t err = gba_mount_sd_card();
    if (err != ESP_OK) {
        return err;
    }

    uint32_t checksum = gba_backup_checksum32();
    if (!force && (checksum == s_backup_checksum)) {
        return ESP_OK;
    }

    FILE *save_file = fopen(GBA_SD_SAVE_TMP_PATH, "wb");
    if (save_file == NULL) {
        ESP_LOGE(TAG, "Failed to open temp save file for writing: %s", GBA_SD_SAVE_TMP_PATH);
        return ESP_FAIL;
    }

    size_t written = fwrite(gamepak_backup, 1, sizeof(gamepak_backup), save_file);
    int write_error = ferror(save_file);

    if ((write_error != 0) || (written != sizeof(gamepak_backup))) {
        fclose(save_file);
        unlink(GBA_SD_SAVE_TMP_PATH);
        ESP_LOGE(TAG,
                 "Failed to write temp save file: %s (written=%u expected=%u)",
                 GBA_SD_SAVE_TMP_PATH,
                 (unsigned int)written,
                 (unsigned int)sizeof(gamepak_backup));
        return ESP_FAIL;
    }

    if (fflush(save_file) != 0) {
        fclose(save_file);
        unlink(GBA_SD_SAVE_TMP_PATH);
        ESP_LOGE(TAG, "Failed to flush temp save file: %s", GBA_SD_SAVE_TMP_PATH);
        return ESP_FAIL;
    }

    int save_fd = fileno(save_file);
    if ((save_fd < 0) || (fsync(save_fd) != 0)) {
        fclose(save_file);
        unlink(GBA_SD_SAVE_TMP_PATH);
        ESP_LOGE(TAG, "Failed to fsync temp save file: %s", GBA_SD_SAVE_TMP_PATH);
        return ESP_FAIL;
    }

    if (fclose(save_file) != 0) {
        unlink(GBA_SD_SAVE_TMP_PATH);
        ESP_LOGE(TAG, "Failed to close temp save file: %s", GBA_SD_SAVE_TMP_PATH);
        return ESP_FAIL;
    }

    unlink(GBA_SD_SAVE_PATH);

    if (rename(GBA_SD_SAVE_TMP_PATH, GBA_SD_SAVE_PATH) != 0) {
        unlink(GBA_SD_SAVE_TMP_PATH);
        ESP_LOGE(TAG, "Failed to publish save file: %s", GBA_SD_SAVE_PATH);
        return ESP_FAIL;
    }

    s_backup_checksum = checksum;
    s_backup_observed_checksum = checksum;
    s_backup_flush_deadline = 0;
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

    const char *rom_source_path = "embedded://pokemon.gba";
    esp_err_t rom_err = gba_load_rom_from_sd_card_to_psram();
    if (rom_err == ESP_OK) {
        rom_source_path = GBA_SD_ROM_PATH;
    } else {
        ESP_LOGW(TAG, "Falling back to embedded ROM source");
        rom_err = gba_decompress_rom_to_psram();
        if (rom_err != ESP_OK) {
            return rom_err;
        }
    }

    size_t bios_size = (size_t)(open_gba_bios_bin_end - open_gba_bios_bin_start);
    if (bios_size != (1024U * 16U)) {
        ESP_LOGE(TAG, "open_gba_bios.bin has invalid size: %u", (unsigned int)bios_size);
        return ESP_FAIL;
    }

    memcpy(bios_rom, open_gba_bios_bin_start, 1024U * 16U);
    memset(gamepak_backup, 0xFF, sizeof(gamepak_backup));
    TickType_t now = xTaskGetTickCount();
    s_backup_checksum = gba_backup_checksum32();
    s_backup_observed_checksum = s_backup_checksum;
    s_backup_flush_deadline = 0;
    s_backup_poll_deadline = now + pdMS_TO_TICKS(GBA_SD_SAVE_POLL_MS);

    libretro_supports_bitmasks = true;
    retro_set_input_state(gba_input_state_cb);

    set_gamepak_source_memory(s_rom_buffer, (u32)s_rom_size);
    if (load_gamepak(NULL,
                     rom_source_path,
                     FEAT_ENABLE,
                     FEAT_DISABLE,
                     SERIAL_MODE_DISABLED) != 0) {
        ESP_LOGE(TAG, "load_gamepak failed");
        return ESP_FAIL;
    }

    esp_err_t save_load_err = gba_load_backup_from_sd_card();
    if ((save_load_err != ESP_OK)
        && (save_load_err != ESP_ERR_NOT_FOUND)
        && (save_load_err != ESP_ERR_TIMEOUT)) {
        ESP_LOGW(TAG, "Failed to load SD save backup: %s", esp_err_to_name(save_load_err));
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

typedef struct {
    bool ready;
    bool connecting;
    bool connected;
    bool scanning;
    char connected_ssid[GBA_WIFI_SSID_LEN + 1];
    size_t ap_count;
    gba_wifi_ap_t aps[GBA_WIFI_AP_LIST_MAX];
    bool list_dirty;
} gba_wifi_snapshot_t;

typedef struct {
    bool ready;
    bool scanning;
    bool list_dirty;
    size_t device_count;
    gba_bt_device_t devices[GBA_BT_SCAN_DEVICE_MAX];
} gba_bt_snapshot_t;

static bool gba_wifi_lock_take(TickType_t timeout)
{
    if (s_wifi_lock == NULL) {
        return false;
    }
    return xSemaphoreTake(s_wifi_lock, timeout) == pdTRUE;
}

static void gba_wifi_lock_give(void)
{
    if (s_wifi_lock != NULL) {
        xSemaphoreGive(s_wifi_lock);
    }
}

static bool gba_bt_lock_take(TickType_t timeout)
{
    if (s_bt_lock == NULL) {
        return false;
    }
    return xSemaphoreTake(s_bt_lock, timeout) == pdTRUE;
}

static void gba_bt_lock_give(void)
{
    if (s_bt_lock != NULL) {
        xSemaphoreGive(s_bt_lock);
    }
}

static void gba_wifi_take_snapshot(gba_wifi_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));

    if (!gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
        return;
    }

    snapshot->ready = s_wifi_ready;
    snapshot->connecting = s_wifi_connecting;
    snapshot->connected = s_wifi_connected;
    snapshot->scanning = s_wifi_scanning;
    snapshot->ap_count = s_wifi_ap_count;
    snapshot->list_dirty = s_wifi_list_dirty;

    if (snapshot->ap_count > GBA_WIFI_AP_LIST_MAX) {
        snapshot->ap_count = GBA_WIFI_AP_LIST_MAX;
    }

    memcpy(snapshot->connected_ssid, s_wifi_connected_ssid, sizeof(snapshot->connected_ssid));
    if (snapshot->ap_count > 0) {
        memcpy(snapshot->aps, s_wifi_ap_list_cache, snapshot->ap_count * sizeof(gba_wifi_ap_t));
    }

    s_wifi_list_dirty = false;
    gba_wifi_lock_give();
}

static void gba_wifi_set_state_flags(bool ready, bool connected, bool connecting)
{
    if (!gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
        return;
    }

    s_wifi_ready = ready;
    s_wifi_connected = connected;
    s_wifi_connecting = connecting;
    s_wifi_state_dirty = true;
    gba_wifi_lock_give();
}

static bool gba_system_time_valid_now(time_t now)
{
    return now >= 1700000000;
}

static void gba_ntp_time_sync_cb(struct timeval *tv)
{
    (void)tv;
    ESP_LOGI(TAG, "NTP synchronized from time.google.com");

    if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
        s_ntp_synced = true;
        s_wifi_state_dirty = true;
        gba_wifi_lock_give();
    }
}

static void gba_ntp_start_if_needed(void)
{
    if (!gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
        return;
    }

    if (s_ntp_started) {
        gba_wifi_lock_give();
        return;
    }

    s_ntp_started = true;
    gba_wifi_lock_give();

    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "time.google.com");
    esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    esp_sntp_set_time_sync_notification_cb(gba_ntp_time_sync_cb);
    esp_sntp_init();

    ESP_LOGI(TAG, "NTP sync started with server time.google.com");
}

static bool gba_is_digit_char(char c)
{
    return (c >= '0') && (c <= '9');
}

static char gba_ascii_upper(char c)
{
    if ((c >= 'a') && (c <= 'z')) {
        return (char)(c - ('a' - 'A'));
    }
    return c;
}

static bool gba_timezone_offset_from_country(const char *country_code, int *out_offset_seconds)
{
    if ((country_code == NULL) || (out_offset_seconds == NULL)) {
        return false;
    }

    char cc[3] = {
        gba_ascii_upper(country_code[0]),
        gba_ascii_upper(country_code[1]),
        '\0',
    };

    if ((cc[0] == '\0') || (cc[1] == '\0')) {
        return false;
    }

    if ((strcmp(cc, "JP") == 0) || (strcmp(cc, "KR") == 0)) {
        *out_offset_seconds = 9 * 3600;
        return true;
    }
    if ((strcmp(cc, "CN") == 0) || (strcmp(cc, "HK") == 0) || (strcmp(cc, "TW") == 0)
        || (strcmp(cc, "SG") == 0) || (strcmp(cc, "MY") == 0)
        || (strcmp(cc, "PH") == 0)) {
        *out_offset_seconds = 8 * 3600;
        return true;
    }
    if ((strcmp(cc, "TH") == 0) || (strcmp(cc, "VN") == 0) || (strcmp(cc, "ID") == 0)) {
        *out_offset_seconds = 7 * 3600;
        return true;
    }
    if (strcmp(cc, "IN") == 0) {
        *out_offset_seconds = (5 * 3600) + (30 * 60);
        return true;
    }
    if (strcmp(cc, "AU") == 0) {
        *out_offset_seconds = 10 * 3600;
        return true;
    }

    return false;
}

static bool gba_parse_utc_offset_seconds(const char *payload, int *out_offset_seconds)
{
    if ((payload == NULL) || (out_offset_seconds == NULL)) {
        return false;
    }

    const char *anchor = strstr(payload, "utc_offset");
    if (anchor == NULL) {
        return false;
    }

    const char *p = anchor;
    while ((*p != '\0') && (*p != '+') && (*p != '-')) {
        p++;
    }
    if ((*p != '+') && (*p != '-')) {
        return false;
    }

    char sign = *p;
    p++;

    if (!gba_is_digit_char(p[0]) || !gba_is_digit_char(p[1]) || (p[2] != ':')
        || !gba_is_digit_char(p[3]) || !gba_is_digit_char(p[4])) {
        return false;
    }

    int hours = (p[0] - '0') * 10 + (p[1] - '0');
    int minutes = (p[3] - '0') * 10 + (p[4] - '0');
    if ((hours > 23) || (minutes > 59)) {
        return false;
    }

    int offset_seconds = (hours * 3600) + (minutes * 60);
    if (sign == '-') {
        offset_seconds = -offset_seconds;
    }

    *out_offset_seconds = offset_seconds;
    return true;
}

static void gba_apply_timezone_offset_seconds(int offset_seconds)
{
    int abs_seconds = (offset_seconds < 0) ? -offset_seconds : offset_seconds;
    int hours = abs_seconds / 3600;
    int minutes = (abs_seconds % 3600) / 60;

    char posix_sign = (offset_seconds >= 0) ? '-' : '+';
    char tz_value[24];
    snprintf(tz_value, sizeof(tz_value), "UTC%c%02d:%02d", posix_sign, hours, minutes);
    setenv("TZ", tz_value, 1);
    tzset();

    ESP_LOGI(TAG,
             "Timezone updated from network: utc_offset=%+03d:%02d, tz=%s",
             offset_seconds / 3600,
             (abs_seconds % 3600) / 60,
             tz_value);
}

static int gba_http_fetch_text(const char *url, char *response, size_t response_size)
{
    if ((url == NULL) || (response == NULL) || (response_size < 2U)) {
        return -1;
    }

    esp_http_client_config_t http_cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 7000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (client == NULL) {
        return -1;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        esp_http_client_cleanup(client);
        return -1;
    }

    (void)esp_http_client_fetch_headers(client);
    int read_len = esp_http_client_read_response(client, response, response_size - 1);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (read_len <= 0) {
        return -1;
    }

    response[read_len] = '\0';
    return read_len;
}

static bool gba_parse_offset_seconds_text(const char *payload, int *out_offset_seconds)
{
    if ((payload == NULL) || (out_offset_seconds == NULL)) {
        return false;
    }

    char *end = NULL;
    long value = strtol(payload, &end, 10);
    if (end == payload) {
        return false;
    }

    if ((value < -86400L) || (value > 86400L)) {
        return false;
    }

    *out_offset_seconds = (int)value;
    return true;
}

static void gba_timezone_lookup_task(void *arg)
{
    (void)arg;

    int offset_seconds = 0;
    bool parsed = false;
    char response[GBA_TZ_HTTP_BUF_SIZE] = {0};

    int read_len = gba_http_fetch_text(GBA_TZ_LOOKUP_URL, response, sizeof(response));
    if (read_len > 0) {
        parsed = gba_parse_utc_offset_seconds(response, &offset_seconds);
    }

    if (!parsed) {
        read_len = gba_http_fetch_text(GBA_TZ_LOOKUP_FALLBACK_URL, response, sizeof(response));
        if (read_len > 0) {
            parsed = gba_parse_offset_seconds_text(response, &offset_seconds);
        }
    }

    if (!parsed) {
        char country_code[4] = {0};
        if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
            memcpy(country_code, s_wifi_country_code, sizeof(country_code));
            gba_wifi_lock_give();
        }

        parsed = gba_timezone_offset_from_country(country_code, &offset_seconds);
        if (parsed) {
            ESP_LOGW(TAG,
                     "Timezone fallback from AP country '%s' -> UTC%+d:%02d",
                     country_code,
                     offset_seconds / 3600,
                     (offset_seconds < 0 ? -offset_seconds : offset_seconds) % 3600 / 60);
        }
    }

    if (parsed) {
        gba_apply_timezone_offset_seconds(offset_seconds);
    } else {
        ESP_LOGW(TAG, "Timezone lookup failed; keeping existing timezone");
    }

    if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
        s_timezone_fetching = false;
        s_wifi_state_dirty = true;
        gba_wifi_lock_give();
    }

    vTaskDelete(NULL);
}

static void gba_start_timezone_lookup_if_needed(void)
{
    if (!gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
        return;
    }

    if (s_timezone_fetching) {
        gba_wifi_lock_give();
        return;
    }

    s_timezone_fetching = true;
    gba_wifi_lock_give();

    if (xTaskCreate(gba_timezone_lookup_task, "tz_lookup", 6144, NULL, 3, NULL) != pdPASS) {
        ESP_LOGW(TAG, "Failed to create timezone lookup task");
        if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
            s_timezone_fetching = false;
            gba_wifi_lock_give();
        }
    }
}

static void gba_wifi_event_handler(void *arg,
                                   esp_event_base_t event_base,
                                   int32_t event_id,
                                   void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            gba_wifi_set_state_flags(true, false, false);
            return;
        }

        if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
                s_wifi_connected = false;
                s_wifi_connecting = false;
                s_wifi_state_dirty = true;
                s_wifi_connected_ssid[0] = '\0';
                gba_wifi_lock_give();
            }
            return;
        }

        if (event_id == WIFI_EVENT_SCAN_DONE) {
            uint16_t ap_num = 0;
            esp_err_t ap_num_err = esp_wifi_scan_get_ap_num(&ap_num);
            if (ap_num_err != ESP_OK) {
                ESP_LOGW(TAG, "esp_wifi_scan_get_ap_num failed: %s", esp_err_to_name(ap_num_err));
                if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
                    s_wifi_scanning = false;
                    s_wifi_scan_pending = false;
                    s_wifi_state_dirty = true;
                    gba_wifi_lock_give();
                }
                return;
            }

            uint16_t fetch_num = ap_num;
            if (fetch_num > GBA_WIFI_AP_LIST_MAX) {
                fetch_num = GBA_WIFI_AP_LIST_MAX;
            }

            wifi_ap_record_t *records = NULL;
            if (fetch_num > 0) {
                records = heap_caps_calloc(fetch_num, sizeof(wifi_ap_record_t), MALLOC_CAP_DEFAULT);
                if (records == NULL) {
                    ESP_LOGW(TAG, "No memory for %u scan records", (unsigned int)fetch_num);
                    fetch_num = 0;
                } else {
                    esp_err_t scan_records_err = esp_wifi_scan_get_ap_records(&fetch_num, records);
                    if (scan_records_err != ESP_OK) {
                        ESP_LOGW(TAG, "esp_wifi_scan_get_ap_records failed: %s", esp_err_to_name(scan_records_err));
                        fetch_num = 0;
                    }
                }
            }

            ESP_LOGI(TAG, "Wi-Fi scan done: ap_num=%u fetched=%u", (unsigned int)ap_num, (unsigned int)fetch_num);

            if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
                s_wifi_ap_count = 0;
                for (uint16_t i = 0; i < fetch_num; i++) {
                    if (records[i].ssid[0] == '\0') {
                        continue;
                    }

                    gba_wifi_ap_t *entry = &s_wifi_ap_list_cache[s_wifi_ap_count];
                    snprintf(entry->ssid, sizeof(entry->ssid), "%s", (const char *)records[i].ssid);
                    entry->rssi = records[i].rssi;
                    entry->authmode = records[i].authmode;

                    s_wifi_ap_count++;
                    if (s_wifi_ap_count >= GBA_WIFI_AP_LIST_MAX) {
                        break;
                    }
                }

                s_wifi_scanning = false;
                s_wifi_scan_pending = false;
                s_wifi_list_dirty = true;
                s_wifi_state_dirty = true;
                gba_wifi_lock_give();
            }

            if (records != NULL) {
                heap_caps_free(records);
            }

            return;
        }

        return;
    }

    if ((event_base == IP_EVENT) && (event_id == IP_EVENT_STA_GOT_IP)) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected, ip=" IPSTR, IP2STR(&event->ip_info.ip));

        wifi_ap_record_t ap_info;
        memset(&ap_info, 0, sizeof(ap_info));
        esp_err_t info_err = esp_wifi_sta_get_ap_info(&ap_info);

        if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
            s_wifi_ready = true;
            s_wifi_connected = true;
            s_wifi_connecting = false;
            s_wifi_state_dirty = true;
            if (info_err == ESP_OK) {
                snprintf(s_wifi_connected_ssid, sizeof(s_wifi_connected_ssid), "%s", (const char *)ap_info.ssid);
                snprintf(s_wifi_country_code,
                         sizeof(s_wifi_country_code),
                         "%c%c",
                         gba_ascii_upper(ap_info.country.cc[0]),
                         gba_ascii_upper(ap_info.country.cc[1]));
            }
            gba_wifi_lock_give();
        }

        gba_ntp_start_if_needed();
        gba_start_timezone_lookup_if_needed();
    }
}

#if GBA_HAS_ESP_HOSTED
static esp_err_t gba_hosted_init_if_needed(void)
{
    esp_err_t err = esp_hosted_init();
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        ESP_LOGE(TAG, "esp_hosted_init failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}
#endif

static esp_err_t gba_wifi_init(void)
{
    if (s_wifi_ready) {
        return ESP_OK;
    }

    if (s_wifi_lock == NULL) {
        s_wifi_lock = xSemaphoreCreateMutex();
        if (s_wifi_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    err = esp_netif_init();
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    err = esp_event_loop_create_default();
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL) {
        return ESP_FAIL;
    }

#if GBA_HAS_ESP_HOSTED
    err = gba_hosted_init_if_needed();
    if (err != ESP_OK) {
        return err;
    }
#endif

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_cfg);
    if ((err != ESP_OK) && (err != ESP_ERR_WIFI_INIT_STATE)) {
        return err;
    }

    if (s_wifi_event_instance == NULL) {
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &gba_wifi_event_handler, NULL, &s_wifi_event_instance));
    }
    if (s_ip_event_instance == NULL) {
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, &gba_wifi_event_handler, NULL, &s_ip_event_instance));
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
        s_wifi_ready = false;
        s_wifi_connected = false;
        s_wifi_connecting = false;
        s_wifi_scanning = false;
        s_wifi_scan_pending = false;
        s_wifi_connected_ssid[0] = '\0';
        s_wifi_state_dirty = true;
        gba_wifi_lock_give();
    }

    return ESP_OK;
}

static esp_err_t gba_wifi_start_scan(void)
{
    if (!gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_wifi_ready) {
        gba_wifi_lock_give();
        ESP_LOGW(TAG, "Scan requested before STA start");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_wifi_scanning) {
        gba_wifi_lock_give();
        return ESP_ERR_INVALID_STATE;
    }

    s_wifi_scanning = true;
    s_wifi_scan_pending = true;
    s_wifi_state_dirty = true;
    gba_wifi_lock_give();

    wifi_scan_config_t scan_cfg = {
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };

    esp_err_t err = esp_wifi_scan_start(&scan_cfg, false);
    ESP_LOGI(TAG, "esp_wifi_scan_start -> %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
            s_wifi_scanning = false;
            s_wifi_scan_pending = false;
            s_wifi_state_dirty = true;
            gba_wifi_lock_give();
        }
    }

    return err;
}

static esp_err_t gba_wifi_connect_to_ap(const char *ssid, const char *password)
{
    if ((ssid == NULL) || (ssid[0] == '\0')) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
        return ESP_ERR_TIMEOUT;
    }

    bool wifi_ready = s_wifi_ready;
    gba_wifi_lock_give();

    if (!wifi_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    wifi_config_t wifi_cfg;
    memset(&wifi_cfg, 0, sizeof(wifi_cfg));
    snprintf((char *)wifi_cfg.sta.ssid, sizeof(wifi_cfg.sta.ssid), "%s", ssid);
    snprintf((char *)wifi_cfg.sta.password, sizeof(wifi_cfg.sta.password), "%s", password ? password : "");
    wifi_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    esp_wifi_disconnect();

    if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
        s_wifi_connecting = true;
        s_wifi_connected = false;
        s_wifi_state_dirty = true;
        snprintf(s_wifi_connected_ssid, sizeof(s_wifi_connected_ssid), "%s", ssid);
        gba_wifi_lock_give();
    }

    return esp_wifi_connect();
}

static void gba_bt_take_snapshot(gba_bt_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));

    if (!gba_bt_lock_take(pdMS_TO_TICKS(50))) {
        return;
    }

    snapshot->ready = s_bt_ready;
    snapshot->scanning = s_bt_scanning;
    snapshot->list_dirty = s_bt_list_dirty;
    snapshot->device_count = s_bt_device_count;
    if (snapshot->device_count > GBA_BT_SCAN_DEVICE_MAX) {
        snapshot->device_count = GBA_BT_SCAN_DEVICE_MAX;
    }
    if (snapshot->device_count > 0) {
        memcpy(snapshot->devices, s_bt_devices, snapshot->device_count * sizeof(gba_bt_device_t));
    }

    gba_bt_lock_give();
}

static void gba_bt_clear_devices_locked(void)
{
    s_bt_device_count = 0;
    memset(s_bt_devices, 0, sizeof(s_bt_devices));
}

#if GBA_BT_STACK_AVAILABLE && GBA_HAS_ESP_HOSTED
typedef struct {
    bool valid;
    int index;
    uint8_t bda[6];
    uint8_t addr_type;
    int8_t rssi;
} gba_bt_name_probe_target_t;

static bool gba_bt_evt_type_connectable(esp_ble_evt_type_t evt_type)
{
    switch (evt_type) {
    case ESP_BLE_EVT_CONN_ADV:
    case ESP_BLE_EVT_CONN_DIR_ADV:
        return true;
    default:
        return false;
    }
}

static void gba_bt_reset_name_probe_locked(void)
{
    s_bt_name_probe_busy = false;
    s_bt_name_probe_succeeded = false;
    s_bt_name_probe_bda_valid = false;
    memset(s_bt_name_probe_bda, 0, sizeof(s_bt_name_probe_bda));
    s_bt_name_probe_index = -1;
    s_bt_name_probe_conn_id = 0;
    s_bt_name_probe_svc_found = false;
    s_bt_name_probe_svc_start_handle = 0;
    s_bt_name_probe_svc_end_handle = 0;
}

static int gba_bt_probe_history_find_locked(const uint8_t *bda)
{
    if (bda == NULL) {
        return -1;
    }

    for (size_t i = 0; i < GBA_BT_NAME_PROBE_HISTORY_SIZE; i++) {
        if (!s_bt_probe_history[i].used) {
            continue;
        }
        if (memcmp(s_bt_probe_history[i].bda, bda, sizeof(s_bt_probe_history[i].bda)) == 0) {
            return (int)i;
        }
    }

    return -1;
}

static int gba_bt_probe_history_acquire_locked(const uint8_t *bda)
{
    int existing = gba_bt_probe_history_find_locked(bda);
    if (existing >= 0) {
        return existing;
    }

    int free_slot = -1;
    uint8_t best_fail_count = UINT8_MAX;
    int replace_slot = 0;

    for (size_t i = 0; i < GBA_BT_NAME_PROBE_HISTORY_SIZE; i++) {
        if (!s_bt_probe_history[i].used) {
            free_slot = (int)i;
            break;
        }
        if (s_bt_probe_history[i].fail_count < best_fail_count) {
            best_fail_count = s_bt_probe_history[i].fail_count;
            replace_slot = (int)i;
        }
    }

    int slot = (free_slot >= 0) ? free_slot : replace_slot;
    memset(&s_bt_probe_history[slot], 0, sizeof(s_bt_probe_history[slot]));
    s_bt_probe_history[slot].used = true;
    memcpy(s_bt_probe_history[slot].bda, bda, sizeof(s_bt_probe_history[slot].bda));
    return slot;
}

static bool gba_bt_probe_in_cooldown_locked(const uint8_t *bda)
{
    int index = gba_bt_probe_history_find_locked(bda);
    if (index < 0) {
        return false;
    }

    if (s_bt_probe_history[index].fail_count == 0U) {
        return false;
    }

    TickType_t now = xTaskGetTickCount();
    return !gba_tick_reached(now, s_bt_probe_history[index].cooldown_until);
}

static void gba_bt_probe_mark_failure_locked(const uint8_t *bda, uint8_t reason)
{
    if (bda == NULL) {
        return;
    }

    int index = gba_bt_probe_history_acquire_locked(bda);
    if (index < 0) {
        return;
    }

    uint8_t fail_count = s_bt_probe_history[index].fail_count;
    if (fail_count < UINT8_MAX) {
        fail_count++;
    }

    uint32_t cooldown_ms = GBA_BT_NAME_PROBE_COOLDOWN_BASE_MS +
                           ((uint32_t)(fail_count > 0U ? (fail_count - 1U) : 0U) *
                            GBA_BT_NAME_PROBE_COOLDOWN_STEP_MS);
    if ((reason == 0x13U) || (reason == 0x3EU)) {
        cooldown_ms += 10000U;
    }
    if (cooldown_ms > GBA_BT_NAME_PROBE_COOLDOWN_MAX_MS) {
        cooldown_ms = GBA_BT_NAME_PROBE_COOLDOWN_MAX_MS;
    }

    s_bt_probe_history[index].fail_count = fail_count;
    s_bt_probe_history[index].cooldown_until = xTaskGetTickCount() + pdMS_TO_TICKS(cooldown_ms);
}

static void gba_bt_probe_mark_success_locked(const uint8_t *bda)
{
    int index = gba_bt_probe_history_find_locked(bda);
    if (index < 0) {
        return;
    }

    s_bt_probe_history[index].fail_count = 0;
    s_bt_probe_history[index].cooldown_until = 0;
}

static bool gba_bt_prepare_name_probe_target_locked(gba_bt_name_probe_target_t *target)
{
    if (target == NULL) {
        return false;
    }

    memset(target, 0, sizeof(*target));
    if (s_bt_name_probe_budget == 0U) {
        return false;
    }

    int best_index = -1;
    int best_rssi = -128;
    int relaxed_index = -1;
    int relaxed_rssi = -128;
    size_t connectable_candidates = 0;
    size_t cooldown_skipped = 0;

    for (size_t i = 0; i < s_bt_device_count; i++) {
        gba_bt_device_t *entry = &s_bt_devices[i];
        if (!entry->connectable) {
            continue;
        }
        if (entry->name_probe_attempted) {
            continue;
        }
        if (entry->name_source >= GBA_BT_NAME_SOURCE_SHORT) {
            continue;
        }
        if (gba_bt_probe_in_cooldown_locked(entry->bda)) {
            cooldown_skipped++;
            continue;
        }

        connectable_candidates++;

        bool is_rpa = (entry->addr_type == BLE_ADDR_TYPE_RPA_PUBLIC) ||
                      (entry->addr_type == BLE_ADDR_TYPE_RPA_RANDOM);

        if (!is_rpa && (entry->rssi >= GBA_BT_NAME_PROBE_MIN_RSSI) &&
            ((best_index < 0) || (entry->rssi > best_rssi))) {
            best_index = (int)i;
            best_rssi = entry->rssi;
        }

        if ((entry->rssi >= GBA_BT_NAME_PROBE_MIN_RSSI_RELAXED) &&
            ((relaxed_index < 0) || (entry->rssi > relaxed_rssi))) {
            relaxed_index = (int)i;
            relaxed_rssi = entry->rssi;
        }
    }

    if (best_index < 0) {
        best_index = relaxed_index;
        best_rssi = relaxed_rssi;
    }

    if (best_index < 0) {
        ESP_LOGI(TAG,
                 "BT name probe skipped: no candidates (devices=%u connectable=%u cooldown=%u budget=%u)",
                 (unsigned int)s_bt_device_count,
                 (unsigned int)connectable_candidates,
                 (unsigned int)cooldown_skipped,
                 (unsigned int)s_bt_name_probe_budget);
        return false;
    }

    s_bt_name_probe_budget--;
    s_bt_name_probe_busy = true;
    s_bt_name_probe_index = best_index;
    s_bt_name_probe_conn_id = 0;
    s_bt_name_probe_svc_found = false;
    s_bt_name_probe_svc_start_handle = 0;
    s_bt_name_probe_svc_end_handle = 0;
    s_bt_name_probe_succeeded = false;
    s_bt_name_probe_bda_valid = true;
    memcpy(s_bt_name_probe_bda, s_bt_devices[best_index].bda, sizeof(s_bt_name_probe_bda));
    s_bt_devices[best_index].name_probe_attempted = true;

    target->valid = true;
    target->index = best_index;
    memcpy(target->bda, s_bt_devices[best_index].bda, sizeof(target->bda));
    target->addr_type = s_bt_devices[best_index].addr_type;
    target->rssi = s_bt_devices[best_index].rssi;
    return true;
}

static void gba_bt_try_kick_name_probe(void)
{
    if (!gba_bt_lock_take(pdMS_TO_TICKS(50))) {
        return;
    }

    if (!s_bt_ready || s_bt_scanning || !s_bt_gattc_ready || (s_bt_gattc_if == ESP_GATT_IF_NONE) ||
        s_bt_name_probe_busy) {
        gba_bt_lock_give();
        return;
    }

    gba_bt_name_probe_target_t target;
    bool has_target = gba_bt_prepare_name_probe_target_locked(&target);
    gba_bt_lock_give();

    if (!has_target || !target.valid) {
        return;
    }

    ESP_LOGI(TAG,
             "BT name probe start %02X:%02X:%02X:%02X:%02X:%02X rssi=%d addr=%s",
             target.bda[0],
             target.bda[1],
             target.bda[2],
             target.bda[3],
             target.bda[4],
             target.bda[5],
             (int)target.rssi,
             gba_bt_addr_type_label(target.addr_type));

    esp_err_t err = esp_ble_gattc_open(s_bt_gattc_if,
                                       target.bda,
                                       (esp_ble_addr_type_t)target.addr_type,
                                       true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "BT name probe open failed for %02X:%02X:%02X:%02X:%02X:%02X: %s",
                 target.bda[0],
                 target.bda[1],
                 target.bda[2],
                 target.bda[3],
                 target.bda[4],
                 target.bda[5],
                 esp_err_to_name(err));

        if (gba_bt_lock_take(pdMS_TO_TICKS(50))) {
            if (s_bt_name_probe_index == target.index) {
                gba_bt_reset_name_probe_locked();
            }
            gba_bt_lock_give();
        }
    }
}

static bool gba_bt_bda_equal(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, 6) == 0;
}

static int gba_bt_find_device_index_locked(const uint8_t *bda)
{
    for (size_t i = 0; i < s_bt_device_count; i++) {
        if (gba_bt_bda_equal(s_bt_devices[i].bda, bda)) {
            return (int)i;
        }
    }
    return -1;
}

static void gba_bt_copy_name(char *dst, size_t dst_size, const uint8_t *src, uint8_t src_len)
{
    if ((dst == NULL) || (dst_size == 0)) {
        return;
    }

    dst[0] = '\0';
    if ((src == NULL) || (src_len == 0)) {
        return;
    }

    size_t n = src_len;
    if (n >= dst_size) {
        n = dst_size - 1;
    }

    memcpy(dst, src, n);
    dst[n] = '\0';
}

static const char *gba_bt_company_name_from_id(uint16_t company_id)
{
    switch (company_id) {
    case 0x004C:
        return "Apple device";
    case 0x0006:
        return "Microsoft device";
    case 0x0075:
        return "Samsung device";
    case 0x00E0:
        return "Google device";
    default:
        return NULL;
    }
}

static const char *gba_bt_addr_type_label(uint8_t addr_type)
{
    switch ((esp_ble_addr_type_t)addr_type) {
    case BLE_ADDR_TYPE_PUBLIC:
        return "PUB";
    case BLE_ADDR_TYPE_RANDOM:
        return "RND";
    case BLE_ADDR_TYPE_RPA_PUBLIC:
        return "RPA-PUB";
    case BLE_ADDR_TYPE_RPA_RANDOM:
        return "RPA-RND";
    default:
        return "UNK";
    }
}

static bool gba_bt_parse_ad_u8(uint8_t *payload,
                               uint16_t payload_len,
                               esp_ble_adv_data_type type,
                               uint8_t *out_value)
{
    if ((payload == NULL) || (payload_len == 0U) || (out_value == NULL)) {
        return false;
    }

    uint8_t field_len = 0;
    uint8_t *field = esp_ble_resolve_adv_data_by_type(payload, payload_len, type, &field_len);
    if ((field == NULL) || (field_len < 1U)) {
        return false;
    }

    *out_value = field[0];
    return true;
}

static bool gba_bt_parse_ad_u16(uint8_t *payload,
                                uint16_t payload_len,
                                esp_ble_adv_data_type type,
                                uint16_t *out_value)
{
    if ((payload == NULL) || (payload_len == 0U) || (out_value == NULL)) {
        return false;
    }

    uint8_t field_len = 0;
    uint8_t *field = esp_ble_resolve_adv_data_by_type(payload, payload_len, type, &field_len);
    if ((field == NULL) || (field_len < 2U)) {
        return false;
    }

    *out_value = (uint16_t)field[0] | ((uint16_t)field[1] << 8);
    return true;
}

static bool gba_bt_parse_service_uuid16(uint8_t *payload,
                                        uint16_t payload_len,
                                        uint16_t *out_service_uuid)
{
    if (gba_bt_parse_ad_u16(payload, payload_len, ESP_BLE_AD_TYPE_16SRV_CMPL, out_service_uuid)) {
        return true;
    }

    return gba_bt_parse_ad_u16(payload,
                               payload_len,
                               ESP_BLE_AD_TYPE_16SRV_PART,
                               out_service_uuid);
}

static void gba_bt_update_adv_metadata_from_payload(gba_bt_device_t *entry,
                                                    uint8_t *payload,
                                                    uint16_t payload_len)
{
    if ((entry == NULL) || (payload == NULL) || (payload_len == 0U)) {
        return;
    }

    uint16_t u16_value = 0;
    uint8_t u8_value = 0;

    if (gba_bt_parse_ad_u16(payload,
                            payload_len,
                            ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE,
                            &u16_value)) {
        entry->has_mfg_company_id = true;
        entry->mfg_company_id = u16_value;
    }

    if (gba_bt_parse_service_uuid16(payload, payload_len, &u16_value)) {
        entry->has_service_uuid16 = true;
        entry->service_uuid16 = u16_value;
    }

    if (gba_bt_parse_ad_u16(payload, payload_len, ESP_BLE_AD_TYPE_SERVICE_DATA, &u16_value)) {
        entry->has_service_data_uuid16 = true;
        entry->service_data_uuid16 = u16_value;
    }

    if (gba_bt_parse_ad_u16(payload, payload_len, ESP_BLE_AD_TYPE_APPEARANCE, &u16_value)) {
        entry->has_appearance = true;
        entry->appearance = u16_value;
    }

    if (gba_bt_parse_ad_u8(payload, payload_len, ESP_BLE_AD_TYPE_TX_PWR, &u8_value)) {
        entry->has_tx_power = true;
        entry->tx_power_dbm = (int8_t)u8_value;
    }

    if (gba_bt_parse_ad_u8(payload, payload_len, ESP_BLE_AD_TYPE_FLAG, &u8_value)) {
        entry->has_flags = true;
        entry->adv_flags = u8_value;
    }
}

static void gba_bt_set_name_with_source(gba_bt_device_t *entry,
                                        gba_bt_name_source_t source,
                                        const uint8_t *name,
                                        uint8_t name_len)
{
    if ((entry == NULL) || (name == NULL) || (name_len == 0)) {
        return;
    }

    if (source < entry->name_source) {
        return;
    }

    gba_bt_copy_name(entry->name, sizeof(entry->name), name, name_len);
    if (entry->name[0] != '\0') {
        entry->name_source = source;
    }
}

static bool gba_bt_try_set_heuristic_name_from_payload(gba_bt_device_t *entry,
                                                        uint8_t *payload,
                                                        uint16_t payload_len)
{
    if ((entry == NULL) || (payload == NULL) || (payload_len == 0)) {
        return false;
    }

    uint8_t field_len = 0;
    uint8_t *field = esp_ble_resolve_adv_data_by_type(payload,
                                                      payload_len,
                                                      ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE,
                                                      &field_len);
    if ((field != NULL) && (field_len >= 2)) {
        uint16_t company_id = (uint16_t)field[0] | ((uint16_t)field[1] << 8);
        char label[GBA_BT_NAME_MAX_LEN];
        const char *company_name = gba_bt_company_name_from_id(company_id);
        if (company_name != NULL) {
            snprintf(label, sizeof(label), "%s", company_name);
        } else {
            snprintf(label, sizeof(label), "MFG 0x%04X", company_id);
        }
        gba_bt_set_name_with_source(entry,
                                    GBA_BT_NAME_SOURCE_HEURISTIC,
                                    (const uint8_t *)label,
                                    (uint8_t)strlen(label));
        return true;
    }

    field = esp_ble_resolve_adv_data_by_type(payload,
                                             payload_len,
                                             ESP_BLE_AD_TYPE_16SRV_CMPL,
                                             &field_len);
    if ((field == NULL) || (field_len < 2)) {
        field = esp_ble_resolve_adv_data_by_type(payload,
                                                 payload_len,
                                                 ESP_BLE_AD_TYPE_16SRV_PART,
                                                 &field_len);
    }
    if ((field != NULL) && (field_len >= 2)) {
        uint16_t service_uuid = (uint16_t)field[0] | ((uint16_t)field[1] << 8);
        char label[GBA_BT_NAME_MAX_LEN];
        snprintf(label, sizeof(label), "Service 0x%04X", service_uuid);
        gba_bt_set_name_with_source(entry,
                                    GBA_BT_NAME_SOURCE_HEURISTIC,
                                    (const uint8_t *)label,
                                    (uint8_t)strlen(label));
        return true;
    }

    return false;
}

static uint8_t *gba_bt_resolve_name_from_payload(uint8_t *payload,
                                                 uint16_t payload_len,
                                                 uint8_t *name_len,
                                                 bool *is_complete_name)
{
    if (name_len == NULL) {
        return NULL;
    }

    *name_len = 0;
    if (is_complete_name != NULL) {
        *is_complete_name = false;
    }

    if ((payload == NULL) || (payload_len == 0)) {
        return NULL;
    }

    uint8_t *name = esp_ble_resolve_adv_data_by_type(payload,
                                                     payload_len,
                                                     ESP_BLE_AD_TYPE_NAME_CMPL,
                                                     name_len);
    if ((name != NULL) && (*name_len > 0)) {
        if (is_complete_name != NULL) {
            *is_complete_name = true;
        }
        return name;
    }

    name = esp_ble_resolve_adv_data_by_type(payload,
                                            payload_len,
                                            ESP_BLE_AD_TYPE_NAME_SHORT,
                                            name_len);
    if ((name != NULL) && (*name_len > 0)) {
        return name;
    }

    return NULL;
}

static void gba_bt_update_device_from_scan_result_locked(const esp_ble_gap_cb_param_t *param)
{
    int index = gba_bt_find_device_index_locked(param->scan_rst.bda);
    if ((index < 0) && (s_bt_device_count < GBA_BT_SCAN_DEVICE_MAX)) {
        index = (int)s_bt_device_count;
        memset(&s_bt_devices[index], 0, sizeof(s_bt_devices[index]));
        s_bt_device_count++;
    }
    if (index < 0) {
        return;
    }

    gba_bt_device_t *entry = &s_bt_devices[index];
    memcpy(entry->bda, param->scan_rst.bda, sizeof(entry->bda));
    entry->rssi = param->scan_rst.rssi;
    entry->addr_type = (uint8_t)param->scan_rst.ble_addr_type;
    if (gba_bt_evt_type_connectable((esp_ble_evt_type_t)param->scan_rst.ble_evt_type)) {
        entry->connectable = true;
    }

    uint8_t name_len = 0;
    bool got_complete_name = false;
    uint16_t adv_data_len = (uint16_t)param->scan_rst.adv_data_len;
    uint16_t scan_rsp_len = (uint16_t)param->scan_rst.scan_rsp_len;
    const uint16_t adv_buf_len = (uint16_t)sizeof(param->scan_rst.ble_adv);
    if (adv_data_len > adv_buf_len) {
        adv_data_len = adv_buf_len;
    }
    if (scan_rsp_len > (adv_buf_len - adv_data_len)) {
        scan_rsp_len = adv_buf_len - adv_data_len;
    }

    if (scan_rsp_len > 0U) {
        entry->has_scan_rsp = true;
    }

    uint16_t merged_len = (uint16_t)(adv_data_len + scan_rsp_len);

    if (merged_len > 0U) {
        gba_bt_update_adv_metadata_from_payload(entry,
                                                (uint8_t *)param->scan_rst.ble_adv,
                                                merged_len);
    }
    if (adv_data_len > 0U) {
        gba_bt_update_adv_metadata_from_payload(entry,
                                                (uint8_t *)param->scan_rst.ble_adv,
                                                adv_data_len);
    }
    if ((scan_rsp_len > 0U) && (adv_data_len < adv_buf_len)) {
        uint8_t *scan_rsp = (uint8_t *)&param->scan_rst.ble_adv[adv_data_len];
        gba_bt_update_adv_metadata_from_payload(entry, scan_rsp, scan_rsp_len);
    }

    uint8_t *name = gba_bt_resolve_name_from_payload((uint8_t *)param->scan_rst.ble_adv,
                                                     merged_len,
                                                     &name_len,
                                                     &got_complete_name);

    if ((name == NULL) && (adv_data_len > 0)) {
        name = gba_bt_resolve_name_from_payload((uint8_t *)param->scan_rst.ble_adv,
                                                adv_data_len,
                                                &name_len,
                                                &got_complete_name);
    }

    if ((name == NULL) && (scan_rsp_len > 0) && (adv_data_len < adv_buf_len)) {
        uint8_t *scan_rsp = (uint8_t *)&param->scan_rst.ble_adv[adv_data_len];
        name = gba_bt_resolve_name_from_payload(scan_rsp,
                                                scan_rsp_len,
                                                &name_len,
                                                &got_complete_name);
    }

    if ((name != NULL) && (name_len > 0)) {
        gba_bt_name_source_t source = got_complete_name ? GBA_BT_NAME_SOURCE_COMPLETE :
                                                           GBA_BT_NAME_SOURCE_SHORT;
        gba_bt_set_name_with_source(entry, source, name, name_len);
    }

    if (entry->name_source < GBA_BT_NAME_SOURCE_SHORT) {
        if (merged_len > 0) {
            (void)gba_bt_try_set_heuristic_name_from_payload(entry,
                                                             (uint8_t *)param->scan_rst.ble_adv,
                                                             merged_len);
        }
        if ((entry->name_source < GBA_BT_NAME_SOURCE_SHORT) && (adv_data_len > 0)) {
            (void)gba_bt_try_set_heuristic_name_from_payload(entry,
                                                             (uint8_t *)param->scan_rst.ble_adv,
                                                             adv_data_len);
        }
        if ((entry->name_source < GBA_BT_NAME_SOURCE_SHORT) && (scan_rsp_len > 0) &&
            (adv_data_len < adv_buf_len)) {
            uint8_t *scan_rsp = (uint8_t *)&param->scan_rst.ble_adv[adv_data_len];
            (void)gba_bt_try_set_heuristic_name_from_payload(entry, scan_rsp, scan_rsp_len);
        }
    }

    s_bt_list_dirty = true;
    s_bt_state_dirty = true;
}

static void gba_bt_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        if (param->scan_param_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGW(TAG, "BT scan param set failed: status=%d", param->scan_param_cmpl.status);
            if (gba_bt_lock_take(pdMS_TO_TICKS(50))) {
                s_bt_scanning = false;
                s_bt_state_dirty = true;
                gba_bt_lock_give();
            }
            return;
        }

        esp_err_t err = esp_ble_gap_start_scanning(GBA_BT_SCAN_DURATION_SEC);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_ble_gap_start_scanning failed: %s", esp_err_to_name(err));
            if (gba_bt_lock_take(pdMS_TO_TICKS(50))) {
                s_bt_scanning = false;
                s_bt_state_dirty = true;
                gba_bt_lock_give();
            }
        }
        return;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGW(TAG, "BT scan start failed: status=%d", param->scan_start_cmpl.status);
            if (gba_bt_lock_take(pdMS_TO_TICKS(50))) {
                s_bt_scanning = false;
                s_bt_state_dirty = true;
                gba_bt_lock_give();
            }
        }
        return;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (!gba_bt_lock_take(pdMS_TO_TICKS(50))) {
            return;
        }

        bool kick_name_probe = false;

        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            gba_bt_update_device_from_scan_result_locked(param);
            gba_bt_lock_give();
            return;
        }

        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
            s_bt_scanning = false;
            s_bt_state_dirty = true;
            s_bt_list_dirty = true;

            size_t local_name_count = 0;
            size_t heuristic_count = 0;
            for (size_t i = 0; i < s_bt_device_count; i++) {
                if (s_bt_devices[i].name_source >= GBA_BT_NAME_SOURCE_SHORT) {
                    local_name_count++;
                } else if (s_bt_devices[i].name_source == GBA_BT_NAME_SOURCE_HEURISTIC) {
                    heuristic_count++;
                }
            }

            size_t unnamed_count = s_bt_device_count;
            if (unnamed_count >= local_name_count) {
                unnamed_count -= local_name_count;
            }
            if (unnamed_count >= heuristic_count) {
                unnamed_count -= heuristic_count;
            }

            ESP_LOGI(TAG,
                     "BT scan complete, devices=%u local_name=%u heuristic=%u unnamed=%u",
                     (unsigned int)s_bt_device_count,
                     (unsigned int)local_name_count,
                     (unsigned int)heuristic_count,
                     (unsigned int)unnamed_count);

            s_bt_name_probe_budget = GBA_BT_NAME_PROBE_MAX_PER_SCAN;
            kick_name_probe = true;
        }

        gba_bt_lock_give();
        if (kick_name_probe) {
            gba_bt_try_kick_name_probe();
        }
        return;
    default:
        return;
    }
}

static void gba_bt_gattc_event_handler(esp_gattc_cb_event_t event,
                                       esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            if (gba_bt_lock_take(pdMS_TO_TICKS(50))) {
                s_bt_gattc_if = gattc_if;
                s_bt_gattc_ready = true;
                gba_bt_lock_give();
            }
            gba_bt_try_kick_name_probe();
        } else {
            ESP_LOGW(TAG, "BT gattc register failed: %d", param->reg.status);
        }
        return;
    case ESP_GATTC_OPEN_EVT:
        if (!gba_bt_lock_take(pdMS_TO_TICKS(50))) {
            return;
        }

        if (!s_bt_name_probe_busy) {
            gba_bt_lock_give();
            if (param->open.status == ESP_GATT_OK) {
                (void)esp_ble_gattc_close(gattc_if, param->open.conn_id);
            }
            return;
        }

        if (param->open.status != ESP_GATT_OK) {
            bool kick_next = false;
            ESP_LOGW(TAG, "BT name probe open failed: status=%d", param->open.status);
            if (s_bt_name_probe_bda_valid) {
                gba_bt_probe_mark_failure_locked(s_bt_name_probe_bda,
                                                 (uint8_t)param->open.status);
            }
            gba_bt_reset_name_probe_locked();
            if (!s_bt_scanning && (s_bt_name_probe_budget > 0U)) {
                kick_next = true;
            }
            gba_bt_lock_give();

            if (kick_next) {
                gba_bt_try_kick_name_probe();
            }
            return;
        }

        s_bt_name_probe_conn_id = param->open.conn_id;
        s_bt_name_probe_svc_found = false;
        s_bt_name_probe_svc_start_handle = 0;
        s_bt_name_probe_svc_end_handle = 0;
        gba_bt_lock_give();

        esp_err_t err = esp_ble_gattc_search_service(gattc_if,
                                                     param->open.conn_id,
                                                     (esp_bt_uuid_t *)&s_bt_gap_service_uuid);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "BT name probe search_service failed: %s", esp_err_to_name(err));
            (void)esp_ble_gattc_close(gattc_if, param->open.conn_id);
        }
        return;
    case ESP_GATTC_SEARCH_RES_EVT:
        if (!gba_bt_lock_take(pdMS_TO_TICKS(50))) {
            return;
        }

        if (s_bt_name_probe_busy && (param->search_res.conn_id == s_bt_name_probe_conn_id) &&
            (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) &&
            (param->search_res.srvc_id.uuid.uuid.uuid16 == s_bt_gap_service_uuid.uuid.uuid16)) {
            s_bt_name_probe_svc_found = true;
            s_bt_name_probe_svc_start_handle = param->search_res.start_handle;
            s_bt_name_probe_svc_end_handle = param->search_res.end_handle;
        }

        gba_bt_lock_give();
        return;
    case ESP_GATTC_SEARCH_CMPL_EVT: {
        uint16_t conn_id = 0;
        uint16_t svc_start = 0;
        uint16_t svc_end = 0;
        bool svc_found = false;

        if (!gba_bt_lock_take(pdMS_TO_TICKS(50))) {
            return;
        }

        if (!s_bt_name_probe_busy || (param->search_cmpl.conn_id != s_bt_name_probe_conn_id)) {
            gba_bt_lock_give();
            return;
        }

        conn_id = param->search_cmpl.conn_id;
        svc_found = s_bt_name_probe_svc_found;
        svc_start = s_bt_name_probe_svc_start_handle;
        svc_end = s_bt_name_probe_svc_end_handle;
        gba_bt_lock_give();

        if (param->search_cmpl.status != ESP_GATT_OK) {
            (void)esp_ble_gattc_close(gattc_if, conn_id);
            return;
        }

        if (!svc_found) {
            (void)esp_ble_gattc_close(gattc_if, conn_id);
            return;
        }

        uint16_t char_count = 0;
        esp_err_t err = esp_ble_gattc_get_attr_count(gattc_if,
                                                     conn_id,
                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                     svc_start,
                                                     svc_end,
                                                     0,
                                                     &char_count);
        if ((err != ESP_OK) || (char_count == 0U)) {
            (void)esp_ble_gattc_close(gattc_if, conn_id);
            return;
        }

        esp_gattc_char_elem_t *char_elems = calloc(char_count, sizeof(*char_elems));
        if (char_elems == NULL) {
            (void)esp_ble_gattc_close(gattc_if, conn_id);
            return;
        }

        uint16_t out_count = char_count;
        err = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                             conn_id,
                                             svc_start,
                                             svc_end,
                                             s_bt_gap_device_name_uuid,
                                             char_elems,
                                             &out_count);
        uint16_t char_handle = 0;
        if ((err == ESP_OK) && (out_count > 0U)) {
            char_handle = char_elems[0].char_handle;
        }
        free(char_elems);

        if (char_handle == 0U) {
            (void)esp_ble_gattc_close(gattc_if, conn_id);
            return;
        }

        err = esp_ble_gattc_read_char(gattc_if, conn_id, char_handle, ESP_GATT_AUTH_REQ_NONE);
        if (err != ESP_OK) {
            (void)esp_ble_gattc_close(gattc_if, conn_id);
        }
        return;
    }
    case ESP_GATTC_READ_CHAR_EVT:
        if (gba_bt_lock_take(pdMS_TO_TICKS(50))) {
            if (s_bt_name_probe_busy && (param->read.conn_id == s_bt_name_probe_conn_id) &&
                (param->read.status == ESP_GATT_OK) && (param->read.value_len > 0U) &&
                (s_bt_name_probe_index >= 0) && ((size_t)s_bt_name_probe_index < s_bt_device_count)) {
                uint16_t raw_len = param->read.value_len;
                uint8_t name_len = (raw_len > UINT8_MAX) ? UINT8_MAX : (uint8_t)raw_len;
                gba_bt_set_name_with_source(&s_bt_devices[s_bt_name_probe_index],
                                            GBA_BT_NAME_SOURCE_COMPLETE,
                                            param->read.value,
                                            name_len);
                if (s_bt_name_probe_bda_valid) {
                    gba_bt_probe_mark_success_locked(s_bt_name_probe_bda);
                }
                s_bt_name_probe_succeeded = true;
                s_bt_list_dirty = true;
                s_bt_state_dirty = true;
            }
            gba_bt_lock_give();
        }
        (void)esp_ble_gattc_close(gattc_if, param->read.conn_id);
        return;
    case ESP_GATTC_CLOSE_EVT: {
        bool kick_next = false;
        if (gba_bt_lock_take(pdMS_TO_TICKS(50))) {
            if (s_bt_name_probe_busy && (param->close.conn_id == s_bt_name_probe_conn_id)) {
                if (!s_bt_name_probe_succeeded && s_bt_name_probe_bda_valid) {
                    gba_bt_probe_mark_failure_locked(s_bt_name_probe_bda, 0U);
                }
                gba_bt_reset_name_probe_locked();
                if (!s_bt_scanning && (s_bt_name_probe_budget > 0U)) {
                    kick_next = true;
                }
            }
            gba_bt_lock_give();
        }

        if (kick_next) {
            gba_bt_try_kick_name_probe();
        }
        return;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
        bool kick_next = false;
        if (gba_bt_lock_take(pdMS_TO_TICKS(50))) {
            if (s_bt_name_probe_busy && (param->disconnect.conn_id == s_bt_name_probe_conn_id)) {
                if (!s_bt_name_probe_succeeded && s_bt_name_probe_bda_valid) {
                    gba_bt_probe_mark_failure_locked(s_bt_name_probe_bda,
                                                     param->disconnect.reason);
                }
                gba_bt_reset_name_probe_locked();
                if (!s_bt_scanning && (s_bt_name_probe_budget > 0U)) {
                    kick_next = true;
                }
            }
            gba_bt_lock_give();
        }

        if (kick_next) {
            gba_bt_try_kick_name_probe();
        }
        return;
    }
    default:
        return;
    }
}
#endif

static esp_err_t gba_bt_init(void)
{
    if (s_bt_lock == NULL) {
        s_bt_lock = xSemaphoreCreateMutex();
        if (s_bt_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_bt_ready) {
        return ESP_OK;
    }

#if GBA_BT_STACK_AVAILABLE && GBA_HAS_ESP_HOSTED
    esp_err_t err = gba_hosted_init_if_needed();
    if (err != ESP_OK) {
        return err;
    }

    err = esp_hosted_connect_to_slave();
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    bool legacy_bt_ctrl_rpc = false;
    err = esp_hosted_bt_controller_init();
    if (err == ESP_ERR_NOT_SUPPORTED) {
        legacy_bt_ctrl_rpc = true;
        ESP_LOGW(TAG,
                 "esp_hosted_bt_controller_init unsupported by co-processor firmware; "
                 "continuing with legacy hosted BT path");
    } else if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    err = esp_hosted_bt_controller_enable();
    if (err == ESP_ERR_NOT_SUPPORTED) {
        legacy_bt_ctrl_rpc = true;
        ESP_LOGW(TAG,
                 "esp_hosted_bt_controller_enable unsupported by co-processor firmware; "
                 "assuming BT controller is already enabled");
    } else if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    if (legacy_bt_ctrl_rpc) {
        ESP_LOGW(TAG, "Recommend updating ESP-Hosted co-processor firmware to v2.5.2+");
    }

    hosted_hci_bluedroid_open();

    err = esp_bluedroid_attach_hci_driver((esp_bluedroid_hci_driver_operations_t *)&s_bt_hci_ops);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    err = esp_bluedroid_init();
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    err = esp_bluedroid_enable();
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    err = esp_ble_gap_register_callback(gba_bt_gap_event_handler);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    err = esp_ble_gattc_register_callback(gba_bt_gattc_event_handler);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    err = esp_ble_gattc_app_register(GBA_BT_NAME_PROBE_APP_ID);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }

    if (gba_bt_lock_take(pdMS_TO_TICKS(50))) {
        s_bt_ready = true;
        s_bt_scanning = false;
        s_bt_state_dirty = true;
        s_bt_list_dirty = true;
        s_bt_name_probe_budget = GBA_BT_NAME_PROBE_MAX_PER_SCAN;
        gba_bt_reset_name_probe_locked();
        gba_bt_clear_devices_locked();
        gba_bt_lock_give();
    }

    return ESP_OK;
#else
    if (gba_bt_lock_take(pdMS_TO_TICKS(50))) {
        s_bt_ready = false;
        s_bt_scanning = false;
        s_bt_state_dirty = true;
        gba_bt_lock_give();
    }
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static esp_err_t gba_bt_start_scan(void)
{
#if GBA_BT_STACK_AVAILABLE && GBA_HAS_ESP_HOSTED
    esp_err_t err = gba_bt_init();
    if (err != ESP_OK) {
        return err;
    }

    if (!gba_bt_lock_take(pdMS_TO_TICKS(50))) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_bt_ready) {
        gba_bt_lock_give();
        return ESP_ERR_INVALID_STATE;
    }

    if (s_bt_scanning) {
        gba_bt_lock_give();
        return ESP_ERR_INVALID_STATE;
    }

    bool cancel_probe = false;
    esp_gatt_if_t cancel_probe_if = ESP_GATT_IF_NONE;
    uint16_t cancel_probe_conn_id = 0;
    if (s_bt_name_probe_busy) {
        if ((s_bt_name_probe_conn_id != 0U) && (s_bt_gattc_if != ESP_GATT_IF_NONE)) {
            cancel_probe = true;
            cancel_probe_if = s_bt_gattc_if;
            cancel_probe_conn_id = s_bt_name_probe_conn_id;
        }
        gba_bt_reset_name_probe_locked();
    }

    s_bt_scanning = true;
    s_bt_state_dirty = true;
    s_bt_name_probe_budget = GBA_BT_NAME_PROBE_MAX_PER_SCAN;
    gba_bt_reset_name_probe_locked();
    gba_bt_clear_devices_locked();
    gba_bt_lock_give();

    if (cancel_probe) {
        ESP_LOGI(TAG, "BT scan requested while probing; cancelling active probe connection");
        (void)esp_ble_gattc_close(cancel_probe_if, cancel_probe_conn_id);
    }

    err = esp_ble_gap_set_scan_params((esp_ble_scan_params_t *)&s_bt_scan_params);
    if ((err == ESP_ERR_INVALID_STATE) || (err == ESP_ERR_NOT_SUPPORTED)) {
        if (err == ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW(TAG, "BT scan params unsupported, trying direct start");
        } else {
            ESP_LOGW(TAG, "BT scan params busy, trying direct start");
        }
        err = esp_ble_gap_start_scanning(GBA_BT_SCAN_DURATION_SEC);
    }

    if (err != ESP_OK) {
        if (gba_bt_lock_take(pdMS_TO_TICKS(50))) {
            s_bt_scanning = false;
            s_bt_state_dirty = true;
            gba_bt_lock_give();
        }
    }

    ESP_LOGI(TAG, "BT scan trigger -> %s", esp_err_to_name(err));
    return err;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void gba_wifi_status_text(const gba_wifi_snapshot_t *snapshot, char *out, size_t out_len)
{
    if ((snapshot == NULL) || (out == NULL) || (out_len == 0U)) {
        return;
    }

    if (!snapshot->ready) {
        snprintf(out, out_len, "Wi-Fi unavailable");
    } else if (snapshot->scanning) {
        snprintf(out, out_len, "Scanning hotspots...");
    } else if (snapshot->connecting) {
        snprintf(out, out_len, "Connecting to %s...", snapshot->connected_ssid[0] ? snapshot->connected_ssid : "AP");
    } else if (snapshot->connected) {
        snprintf(out, out_len, "Connected: %s", snapshot->connected_ssid[0] ? snapshot->connected_ssid : "(hidden SSID)");
    } else {
        snprintf(out, out_len, "Wi-Fi disconnected");
    }
}

static void gba_update_time_label_locked(void)
{
    if (s_info_time_label == NULL) {
        return;
    }

    time_t now = 0;
    time(&now);

    if (!gba_system_time_valid_now(now)) {
        lv_label_set_text(s_info_time_label, s_ntp_started ? "Time syncing..." : "Time --:--:--");
        return;
    }

    if (gba_wifi_lock_take(pdMS_TO_TICKS(10))) {
        s_ntp_synced = true;
        gba_wifi_lock_give();
    }

    struct tm tm_info;
    localtime_r(&now, &tm_info);

    char time_text[32];
    strftime(time_text, sizeof(time_text), "%H:%M:%S", &tm_info);
    lv_label_set_text(s_info_time_label, time_text);
}

static void gba_update_bt_popup_text_locked(void)
{
    if (s_bt_popup_status_label == NULL) {
        return;
    }

    gba_bt_snapshot_t snapshot;
    gba_bt_take_snapshot(&snapshot);

#if GBA_BT_STACK_AVAILABLE && GBA_HAS_ESP_HOSTED
    if (!snapshot.ready) {
        lv_label_set_text(s_bt_popup_status_label, "Bluetooth is initializing...");
        if (s_bt_popup_scan_btn_label != NULL) {
            lv_label_set_text(s_bt_popup_scan_btn_label, "Start");
        }
    } else if (snapshot.scanning) {
        lv_label_set_text(s_bt_popup_status_label, "Scanning nearby devices...");
        if (s_bt_popup_scan_btn_label != NULL) {
            lv_label_set_text(s_bt_popup_scan_btn_label, "Scanning");
        }
    } else if (snapshot.device_count > 0) {
        lv_label_set_text_fmt(s_bt_popup_status_label, "Found %u device(s)", (unsigned int)snapshot.device_count);
        if (s_bt_popup_scan_btn_label != NULL) {
            lv_label_set_text(s_bt_popup_scan_btn_label, "Scan Again");
        }
    } else {
        lv_label_set_text(s_bt_popup_status_label, "No devices found yet");
        if (s_bt_popup_scan_btn_label != NULL) {
            lv_label_set_text(s_bt_popup_scan_btn_label, "Scan");
        }
    }
#else
    lv_label_set_text(s_bt_popup_status_label, "Bluetooth backend unavailable");
    if (s_bt_popup_scan_btn_label != NULL) {
        lv_label_set_text(s_bt_popup_scan_btn_label, "Unavailable");
    }
#endif
}

static void gba_close_bt_popup_locked(void)
{
    if (s_bt_popup != NULL) {
        lv_obj_delete(s_bt_popup);
        s_bt_popup = NULL;
        s_bt_popup_status_label = NULL;
        s_bt_popup_scan_btn_label = NULL;
        s_bt_device_list = NULL;
    }
}

static void gba_close_wifi_password_popup_locked(void)
{
    if (s_wifi_password_popup != NULL) {
        if ((s_wifi_password_keyboard != NULL) && (s_wifi_password_ta != NULL)) {
            lv_keyboard_set_textarea(s_wifi_password_keyboard, NULL);
        }
        lv_obj_delete(s_wifi_password_popup);
        s_wifi_password_popup = NULL;
        s_wifi_password_ta = NULL;
        s_wifi_password_keyboard = NULL;
    }
}

static void gba_close_wifi_popup_locked(void)
{
    gba_close_wifi_password_popup_locked();

    if (s_wifi_popup != NULL) {
        lv_obj_delete(s_wifi_popup);
        s_wifi_popup = NULL;
        s_wifi_popup_status_label = NULL;
        s_wifi_ap_list = NULL;
    }
}

static void gba_wifi_password_cancel_event_cb(lv_event_t *event)
{
    (void)event;
    gba_close_wifi_password_popup_locked();
}

static void gba_wifi_password_connect_event_cb(lv_event_t *event)
{
    (void)event;
    if ((s_wifi_password_ta == NULL) || (s_wifi_selected_ssid[0] == '\0')) {
        return;
    }

    const char *password = lv_textarea_get_text(s_wifi_password_ta);
    esp_err_t err = gba_wifi_connect_to_ap(s_wifi_selected_ssid, password);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Wi-Fi connect request sent for SSID '%s'", s_wifi_selected_ssid);
    }

    gba_close_wifi_password_popup_locked();
}

static void gba_wifi_keyboard_event_cb(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);
    if ((code == LV_EVENT_READY) || (code == LV_EVENT_CANCEL)) {
        gba_close_wifi_password_popup_locked();
    }
}

static void gba_wifi_popup_close_event_cb(lv_event_t *event)
{
    (void)event;
    gba_close_wifi_popup_locked();
}

static void gba_wifi_scan_event_cb(lv_event_t *event)
{
    (void)event;
    esp_err_t err = gba_wifi_start_scan();
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        ESP_LOGW(TAG, "Wi-Fi scan failed: %s", esp_err_to_name(err));
    }
}

static void gba_bt_popup_close_event_cb(lv_event_t *event)
{
    (void)event;
    gba_close_bt_popup_locked();
}

static void gba_bt_scan_event_cb(lv_event_t *event)
{
    (void)event;
    esp_err_t err = gba_bt_start_scan();
    if (err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Bluetooth scan unsupported by co-processor firmware/config");
    } else if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Bluetooth scan request ignored (busy/not ready)");
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "Bluetooth scan failed: %s", esp_err_to_name(err));
    }
    gba_refresh_info_ui_locked();
}

static void gba_wifi_ap_selected_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    size_t index = (size_t)(uintptr_t)lv_event_get_user_data(event);
    char ssid[GBA_WIFI_SSID_LEN + 1] = {0};

    if (gba_wifi_lock_take(pdMS_TO_TICKS(50))) {
        if (index < s_wifi_ap_count) {
            snprintf(ssid, sizeof(ssid), "%s", s_wifi_ap_list_cache[index].ssid);
            snprintf(s_wifi_selected_ssid, sizeof(s_wifi_selected_ssid), "%s", s_wifi_ap_list_cache[index].ssid);
        }
        gba_wifi_lock_give();
    }

    if (ssid[0] == '\0') {
        return;
    }

    gba_close_wifi_password_popup_locked();

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LCD_H_RES, LCD_V_RES);
    lv_obj_center(overlay);
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);
    gba_disable_scroll(overlay);
    s_wifi_password_popup = overlay;

    lv_obj_t *panel = lv_obj_create(overlay);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, LCD_H_RES - 52, 332);
    lv_obj_align(panel, LV_ALIGN_TOP_MID, 0, 86);
    lv_obj_set_style_radius(panel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x273246), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(panel, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x92A2C4), LV_PART_MAIN | LV_STATE_DEFAULT);
    gba_disable_scroll(panel);

    lv_obj_t *title = lv_label_create(panel);
    lv_label_set_text_fmt(title, "Connect to %s", ssid);
    lv_obj_set_style_text_color(title, lv_color_hex(0xE5EEFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 14, 12);

    s_wifi_password_ta = lv_textarea_create(panel);
    lv_obj_set_size(s_wifi_password_ta, LCD_H_RES - 80, 46);
    lv_obj_set_style_radius(s_wifi_password_ta, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(s_wifi_password_ta, lv_color_hex(0x111722), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(s_wifi_password_ta, lv_color_hex(0xE5EEFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(s_wifi_password_ta, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(s_wifi_password_ta, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(s_wifi_password_ta, LV_ALIGN_TOP_LEFT, 14, 46);
    lv_textarea_set_one_line(s_wifi_password_ta, true);
    lv_textarea_set_password_mode(s_wifi_password_ta, true);
    lv_textarea_set_placeholder_text(s_wifi_password_ta, "Password");

    lv_obj_t *connect_btn = lv_button_create(panel);
    lv_obj_set_size(connect_btn, 132, 36);
    lv_obj_align(connect_btn, LV_ALIGN_TOP_LEFT, 14, 100);
    lv_obj_set_style_bg_color(connect_btn, lv_color_hex(0x2D8C5D), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_t *connect_lbl = lv_label_create(connect_btn);
    lv_label_set_text(connect_lbl, "Connect");
    lv_obj_center(connect_lbl);

    lv_obj_t *cancel_btn = lv_button_create(panel);
    lv_obj_set_size(cancel_btn, 132, 36);
    lv_obj_align(cancel_btn, LV_ALIGN_TOP_LEFT, 158, 100);
    lv_obj_set_style_bg_color(cancel_btn, lv_color_hex(0x5D6472), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_center(cancel_lbl);

    lv_obj_add_event_cb(cancel_btn, gba_wifi_password_cancel_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(connect_btn, gba_wifi_password_connect_event_cb, LV_EVENT_CLICKED, NULL);

    s_wifi_password_keyboard = lv_keyboard_create(overlay);
    lv_obj_set_size(s_wifi_password_keyboard, LCD_H_RES, 236);
    lv_obj_align(s_wifi_password_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(s_wifi_password_keyboard, s_wifi_password_ta);

    lv_obj_add_state(s_wifi_password_ta, LV_STATE_FOCUSED);
    lv_obj_send_event(s_wifi_password_ta, LV_EVENT_FOCUSED, NULL);
    lv_textarea_set_cursor_pos(s_wifi_password_ta, LV_TEXTAREA_CURSOR_LAST);
    lv_obj_invalidate(s_wifi_password_ta);

    lv_obj_add_event_cb(s_wifi_password_keyboard, gba_wifi_keyboard_event_cb, LV_EVENT_ALL, NULL);
}

static void gba_rebuild_wifi_ap_list_locked(void)
{
    if (s_wifi_ap_list == NULL) {
        return;
    }

    gba_wifi_snapshot_t snapshot;
    gba_wifi_take_snapshot(&snapshot);

    lv_obj_clean(s_wifi_ap_list);
    if (snapshot.ap_count == 0U) {
        lv_obj_t *empty = lv_list_add_text(s_wifi_ap_list, "No hotspots found");
        lv_obj_set_style_text_color(empty, lv_color_hex(0xD0D7E8), LV_PART_MAIN | LV_STATE_DEFAULT);
        return;
    }

    for (size_t i = 0; i < snapshot.ap_count; i++) {
        char row_text[88];
        const char *secure_suffix = (snapshot.aps[i].authmode == WIFI_AUTH_OPEN) ? "" : " [secured]";
        snprintf(row_text,
                 sizeof(row_text),
                 "%s (%d dBm)%s",
                 snapshot.aps[i].ssid,
                 (int)snapshot.aps[i].rssi,
                 secure_suffix);

        lv_obj_t *item = lv_list_add_button(s_wifi_ap_list, LV_SYMBOL_WIFI, row_text);
        lv_obj_add_event_cb(item, gba_wifi_ap_selected_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }
}

static void gba_rebuild_bt_device_list_locked(void)
{
    if (s_bt_device_list == NULL) {
        return;
    }

    gba_bt_snapshot_t snapshot;
    gba_bt_take_snapshot(&snapshot);

    lv_obj_clean(s_bt_device_list);

#if GBA_BT_STACK_AVAILABLE && GBA_HAS_ESP_HOSTED
    if (!snapshot.ready) {
        lv_obj_t *line = lv_list_add_text(s_bt_device_list, "Bluetooth is initializing...");
        lv_obj_set_style_text_color(line, lv_color_hex(0xD0D7E8), LV_PART_MAIN | LV_STATE_DEFAULT);
    } else if ((snapshot.device_count == 0U) && snapshot.scanning) {
        lv_obj_t *line = lv_list_add_text(s_bt_device_list, "Scanning...");
        lv_obj_set_style_text_color(line, lv_color_hex(0xD0D7E8), LV_PART_MAIN | LV_STATE_DEFAULT);
    } else if (snapshot.device_count == 0U) {
        lv_obj_t *line = lv_list_add_text(s_bt_device_list, "No devices discovered");
        lv_obj_set_style_text_color(line, lv_color_hex(0xD0D7E8), LV_PART_MAIN | LV_STATE_DEFAULT);
    } else {
        for (size_t i = 0; i < snapshot.device_count; i++) {
            char row_text[256];
            char addr_text[24];
            char meta_text[196];
            char mfg_part[20] = "";
            char svc_part[20] = "";
            char svc_data_part[20] = "";
            char tx_part[16] = "";
            char app_part[20] = "";
            char flags_part[16] = "";
            char scan_rsp_part[8] = "";
            char hint_part[56] = "";

            bool has_local_name = (snapshot.devices[i].name_source >= GBA_BT_NAME_SOURCE_SHORT) &&
                                  (snapshot.devices[i].name[0] != '\0');
            const char *display_name = has_local_name ? snapshot.devices[i].name : "Unnamed device";
            const char *addr_type = gba_bt_addr_type_label(snapshot.devices[i].addr_type);

            snprintf(addr_text,
                     sizeof(addr_text),
                     "%02X:%02X:%02X:%02X:%02X:%02X",
                     snapshot.devices[i].bda[0],
                     snapshot.devices[i].bda[1],
                     snapshot.devices[i].bda[2],
                     snapshot.devices[i].bda[3],
                     snapshot.devices[i].bda[4],
                     snapshot.devices[i].bda[5]);

            if (snapshot.devices[i].has_mfg_company_id) {
                snprintf(mfg_part,
                         sizeof(mfg_part),
                         " MFG:%04X",
                         (unsigned int)snapshot.devices[i].mfg_company_id);
            }
            if (snapshot.devices[i].has_service_uuid16) {
                snprintf(svc_part,
                         sizeof(svc_part),
                         " SVC:%04X",
                         (unsigned int)snapshot.devices[i].service_uuid16);
            }
            if (snapshot.devices[i].has_service_data_uuid16) {
                snprintf(svc_data_part,
                         sizeof(svc_data_part),
                         " SD:%04X",
                         (unsigned int)snapshot.devices[i].service_data_uuid16);
            }
            if (snapshot.devices[i].has_tx_power) {
                snprintf(tx_part, sizeof(tx_part), " TX:%d", (int)snapshot.devices[i].tx_power_dbm);
            }
            if (snapshot.devices[i].has_appearance) {
                snprintf(app_part,
                         sizeof(app_part),
                         " APP:%04X",
                         (unsigned int)snapshot.devices[i].appearance);
            }
            if (snapshot.devices[i].has_flags) {
                snprintf(flags_part,
                         sizeof(flags_part),
                         " FL:%02X",
                         (unsigned int)snapshot.devices[i].adv_flags);
            }
            if (snapshot.devices[i].has_scan_rsp) {
                snprintf(scan_rsp_part, sizeof(scan_rsp_part), " SR");
            }

            if (!has_local_name && (snapshot.devices[i].name_source == GBA_BT_NAME_SOURCE_HEURISTIC) &&
                (snapshot.devices[i].name[0] != '\0')) {
                snprintf(hint_part, sizeof(hint_part), " H:%s", snapshot.devices[i].name);
            }

            snprintf(meta_text,
                     sizeof(meta_text),
                     "%s %s%s%s%s%s%s%s%s",
                     snapshot.devices[i].connectable ? "CONN" : "NONCONN",
                     addr_type,
                     scan_rsp_part,
                     mfg_part,
                     svc_part,
                     svc_data_part,
                     tx_part,
                     app_part,
                     flags_part);

            if (hint_part[0] != '\0') {
                size_t base_len = strlen(meta_text);
                if (base_len < sizeof(meta_text) - 1U) {
                    snprintf(meta_text + base_len,
                             sizeof(meta_text) - base_len,
                             "%s",
                             hint_part);
                }
            }

            snprintf(row_text,
                     sizeof(row_text),
                     "%s (%d dBm)\n%s\n%s",
                     display_name,
                     (int)snapshot.devices[i].rssi,
                     addr_text,
                     meta_text);
            lv_list_add_button(s_bt_device_list, LV_SYMBOL_BLUETOOTH, row_text);
        }
    }
#else
    lv_obj_t *line = lv_list_add_text(s_bt_device_list, "Bluetooth backend unavailable");
    lv_obj_set_style_text_color(line, lv_color_hex(0xD0D7E8), LV_PART_MAIN | LV_STATE_DEFAULT);
#endif

    if (gba_bt_lock_take(pdMS_TO_TICKS(20))) {
        s_bt_list_dirty = false;
        gba_bt_lock_give();
    }
}

static void gba_refresh_info_ui_locked(void)
{
    gba_wifi_snapshot_t snapshot;
    gba_wifi_take_snapshot(&snapshot);

    if (s_wifi_icon_label != NULL) {
        const char *wifi_symbol = LV_SYMBOL_WIFI;
        lv_color_t wifi_color = lv_color_hex(0xAAB3C7);

        if (!snapshot.ready) {
            wifi_symbol = LV_SYMBOL_WARNING;
            wifi_color = lv_color_hex(0xF39A72);
        } else if (snapshot.scanning || snapshot.connecting) {
            wifi_symbol = LV_SYMBOL_REFRESH;
            wifi_color = lv_color_hex(0xFFD18A);
        } else if (snapshot.connected) {
            wifi_symbol = LV_SYMBOL_WIFI;
            wifi_color = lv_color_hex(0x8FE39D);
        }

        lv_label_set_text(s_wifi_icon_label, wifi_symbol);
        lv_obj_set_style_text_color(s_wifi_icon_label, wifi_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    gba_update_time_label_locked();

    gba_bt_snapshot_t bt_snapshot;
    gba_bt_take_snapshot(&bt_snapshot);

    if (s_bt_icon_label != NULL) {
        lv_obj_set_style_text_color(s_bt_icon_label,
#if GBA_BT_STACK_AVAILABLE && GBA_HAS_ESP_HOSTED
                                    bt_snapshot.scanning ? lv_color_hex(0xFFD18A)
                                                         : (bt_snapshot.ready ? lv_color_hex(0x8FE39D)
                                                                              : lv_color_hex(0xAAB3C7)),
#else
                                    lv_color_hex(0xF39A72),
#endif
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    if (s_wifi_popup_status_label != NULL) {
        char wifi_status[96];
        gba_wifi_status_text(&snapshot, wifi_status, sizeof(wifi_status));
        lv_label_set_text(s_wifi_popup_status_label, wifi_status);
    }

    if ((s_wifi_ap_list != NULL) && snapshot.list_dirty) {
        gba_rebuild_wifi_ap_list_locked();
    }

    if (s_bt_popup != NULL) {
        gba_update_bt_popup_text_locked();
        if (bt_snapshot.list_dirty || bt_snapshot.scanning) {
            gba_rebuild_bt_device_list_locked();
        }
    }
}

static void gba_info_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    gba_refresh_info_ui_locked();
}

static void gba_open_wifi_popup_locked(void)
{
    if (s_wifi_popup != NULL) {
        return;
    }

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LCD_H_RES, LCD_V_RES);
    lv_obj_center(overlay);
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_50, LV_PART_MAIN | LV_STATE_DEFAULT);
    gba_disable_scroll(overlay);
    s_wifi_popup = overlay;

    lv_obj_t *panel = lv_obj_create(overlay);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, LCD_H_RES - 40, LCD_V_RES - 170);
    lv_obj_align(panel, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_style_radius(panel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x243147), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(panel, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x7F93BA), LV_PART_MAIN | LV_STATE_DEFAULT);
    gba_disable_scroll(panel);

    lv_obj_t *title = lv_label_create(panel);
    lv_label_set_text(title, "Wi-Fi Hotspots");
    lv_obj_set_style_text_color(title, lv_color_hex(0xE6EEFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 14, 12);

    lv_obj_t *close_btn = lv_button_create(panel);
    lv_obj_set_size(close_btn, 36, 30);
    lv_obj_align(close_btn, LV_ALIGN_TOP_RIGHT, -12, 8);
    lv_obj_t *close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, LV_SYMBOL_CLOSE);
    lv_obj_center(close_lbl);
    lv_obj_add_event_cb(close_btn, gba_wifi_popup_close_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *refresh_btn = lv_button_create(panel);
    lv_obj_set_size(refresh_btn, 116, 34);
    lv_obj_align(refresh_btn, LV_ALIGN_TOP_RIGHT, -56, 48);
    lv_obj_set_style_bg_color(refresh_btn, lv_color_hex(0x3F5576), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_t *refresh_lbl = lv_label_create(refresh_btn);
    lv_label_set_text_fmt(refresh_lbl, "%s Scan", LV_SYMBOL_REFRESH);
    lv_obj_center(refresh_lbl);
    lv_obj_add_event_cb(refresh_btn, gba_wifi_scan_event_cb, LV_EVENT_CLICKED, NULL);

    s_wifi_popup_status_label = lv_label_create(panel);
    lv_obj_set_width(s_wifi_popup_status_label, LCD_H_RES - 68);
    lv_obj_set_style_text_color(s_wifi_popup_status_label, lv_color_hex(0xBFD1EE), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(s_wifi_popup_status_label, LV_ALIGN_TOP_LEFT, 14, 52);

    s_wifi_ap_list = lv_list_create(panel);
    lv_obj_set_size(s_wifi_ap_list, LCD_H_RES - 60, LCD_V_RES - 272);
    lv_obj_set_style_pad_all(s_wifi_ap_list, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(s_wifi_ap_list, LV_ALIGN_TOP_LEFT, 10, 92);
    lv_obj_set_style_bg_color(s_wifi_ap_list, lv_color_hex(0x141C2C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(s_wifi_ap_list, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(s_wifi_ap_list, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    gba_rebuild_wifi_ap_list_locked();
    gba_refresh_info_ui_locked();

    esp_err_t scan_err = gba_wifi_start_scan();
    if ((scan_err != ESP_OK) && (scan_err != ESP_ERR_INVALID_STATE)) {
        ESP_LOGW(TAG, "Wi-Fi scan failed: %s", esp_err_to_name(scan_err));
    }
}

static void gba_open_bt_popup_locked(void)
{
    if (s_bt_popup != NULL) {
        return;
    }

    lv_obj_t *overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LCD_H_RES, LCD_V_RES);
    lv_obj_center(overlay);
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_50, LV_PART_MAIN | LV_STATE_DEFAULT);
    gba_disable_scroll(overlay);
    s_bt_popup = overlay;

    lv_obj_t *panel = lv_obj_create(overlay);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, LCD_H_RES - 56, LCD_V_RES - 220);
    lv_obj_align(panel, LV_ALIGN_TOP_MID, 0, 76);
    lv_obj_set_style_radius(panel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x243147), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(panel, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x7F93BA), LV_PART_MAIN | LV_STATE_DEFAULT);
    gba_disable_scroll(panel);

    lv_obj_t *title = lv_label_create(panel);
    lv_label_set_text(title, "Bluetooth");
    lv_obj_set_style_text_color(title, lv_color_hex(0xE6EEFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 14, 14);

    lv_obj_t *close_btn = lv_button_create(panel);
    lv_obj_set_size(close_btn, 36, 30);
    lv_obj_align(close_btn, LV_ALIGN_TOP_RIGHT, -12, 8);
    lv_obj_t *close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, LV_SYMBOL_CLOSE);
    lv_obj_center(close_lbl);
    lv_obj_add_event_cb(close_btn, gba_bt_popup_close_event_cb, LV_EVENT_CLICKED, NULL);

    s_bt_popup_status_label = lv_label_create(panel);
    lv_obj_set_width(s_bt_popup_status_label, LCD_H_RES - 130);
    lv_obj_align(s_bt_popup_status_label, LV_ALIGN_TOP_LEFT, 14, 58);
    lv_obj_set_style_text_color(s_bt_popup_status_label, lv_color_hex(0xC8D5EE), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *scan_btn = lv_button_create(panel);
    lv_obj_set_size(scan_btn, 138, 36);
    lv_obj_align(scan_btn, LV_ALIGN_TOP_RIGHT, -14, 50);
    lv_obj_set_style_bg_color(scan_btn, lv_color_hex(0x3F5576), LV_PART_MAIN | LV_STATE_DEFAULT);
    s_bt_popup_scan_btn_label = lv_label_create(scan_btn);
    lv_obj_center(s_bt_popup_scan_btn_label);
    lv_obj_add_event_cb(scan_btn, gba_bt_scan_event_cb, LV_EVENT_CLICKED, NULL);

    s_bt_device_list = lv_list_create(panel);
    lv_obj_set_size(s_bt_device_list, LCD_H_RES - 92, LCD_V_RES - 340);
    lv_obj_set_style_pad_all(s_bt_device_list, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(s_bt_device_list, LV_ALIGN_TOP_LEFT, 12, 98);
    lv_obj_set_style_bg_color(s_bt_device_list, lv_color_hex(0x141C2C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(s_bt_device_list, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(s_bt_device_list, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    gba_rebuild_bt_device_list_locked();
    gba_update_bt_popup_text_locked();

    esp_err_t scan_err = gba_bt_start_scan();
    if (scan_err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG,
                 "Initial BT scan unsupported; verify C6 BT firmware and hosted BT settings");
    } else if ((scan_err != ESP_OK) && (scan_err != ESP_ERR_INVALID_STATE)) {
        ESP_LOGW(TAG, "Initial BT scan failed: %s", esp_err_to_name(scan_err));
    }
}

static void gba_info_item_click_event_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    uintptr_t item = (uintptr_t)lv_event_get_user_data(event);
    if (item == GBA_INFO_ITEM_WIFI) {
        gba_open_wifi_popup_locked();
    } else if (item == GBA_INFO_ITEM_BLUETOOTH) {
        gba_open_bt_popup_locked();
    }
}

static void gba_create_info_bar(lv_obj_t *screen)
{
    lv_obj_t *bar = lv_obj_create(screen);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, LCD_H_RES - (GBA_INFO_BAR_PAD_X * 2), GBA_INFO_BAR_HEIGHT);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 12);
    lv_obj_set_style_radius(bar, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x1A2436), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(bar, LV_OPA_70, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(bar, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(bar, lv_color_hex(0x7A89A7), LV_PART_MAIN | LV_STATE_DEFAULT);
    gba_disable_scroll(bar);

    lv_obj_t *title = lv_label_create(bar);
    lv_label_set_text(title, "Status");
    lv_obj_set_style_text_color(title, lv_color_hex(0xDDE7F8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 12, 0);

    s_info_time_label = lv_label_create(bar);
    lv_label_set_text(s_info_time_label, "Time --:--:--");
    lv_obj_set_style_text_color(s_info_time_label, lv_color_hex(0xDDE7F8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(s_info_time_label, LV_ALIGN_CENTER, 10, 0);

    s_bt_icon_btn = lv_button_create(bar);
    lv_obj_set_size(s_bt_icon_btn, 32, 32);
    lv_obj_align(s_bt_icon_btn, LV_ALIGN_RIGHT_MID, -8, 0);
    lv_obj_set_style_radius(s_bt_icon_btn, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(s_bt_icon_btn, lv_color_hex(0x2F3F5B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(s_bt_icon_btn, gba_info_item_click_event_cb, LV_EVENT_CLICKED, (void *)GBA_INFO_ITEM_BLUETOOTH);

    s_bt_icon_label = lv_label_create(s_bt_icon_btn);
    lv_label_set_text(s_bt_icon_label, LV_SYMBOL_BLUETOOTH);
    lv_obj_center(s_bt_icon_label);

    s_wifi_icon_btn = lv_button_create(bar);
    lv_obj_set_size(s_wifi_icon_btn, 32, 32);
    lv_obj_align_to(s_wifi_icon_btn, s_bt_icon_btn, LV_ALIGN_OUT_LEFT_MID, -8, 0);
    lv_obj_set_style_radius(s_wifi_icon_btn, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(s_wifi_icon_btn, lv_color_hex(0x2F3F5B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(s_wifi_icon_btn, gba_info_item_click_event_cb, LV_EVENT_CLICKED, (void *)GBA_INFO_ITEM_WIFI);

    s_wifi_icon_label = lv_label_create(s_wifi_icon_btn);
    lv_label_set_text(s_wifi_icon_label, LV_SYMBOL_WIFI);
    lv_obj_center(s_wifi_icon_label);

    if (s_info_timer == NULL) {
        s_info_timer = lv_timer_create(gba_info_timer_cb, 500, NULL);
    }

    gba_refresh_info_ui_locked();
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

    gba_create_info_bar(screen);

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

            if ((s_backup_poll_deadline != 0U) && gba_tick_reached(now_tick, s_backup_poll_deadline)) {
                uint32_t observed_checksum = gba_backup_checksum32();
                if (observed_checksum != s_backup_observed_checksum) {
                    s_backup_observed_checksum = observed_checksum;
                    s_backup_flush_deadline = now_tick + pdMS_TO_TICKS(GBA_SD_SAVE_FLUSH_MS);
                }
                s_backup_poll_deadline = now_tick + pdMS_TO_TICKS(GBA_SD_SAVE_POLL_MS);
            }

            if ((s_backup_flush_deadline != 0U) && gba_tick_reached(now_tick, s_backup_flush_deadline)) {
                esp_err_t save_flush_err = gba_flush_backup_to_sd_card(false);
                if (save_flush_err == ESP_OK) {
                    s_backup_flush_deadline = 0;
                } else {
                    if ((save_flush_err != ESP_ERR_TIMEOUT) && (save_flush_err != ESP_ERR_NOT_FOUND)) {
                        ESP_LOGW(TAG, "Failed to flush SD save backup: %s", esp_err_to_name(save_flush_err));
                    }
                    s_backup_flush_deadline = now_tick + pdMS_TO_TICKS(GBA_SD_SAVE_FLUSH_MS);
                }
            }

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

    esp_err_t wifi_err = gba_wifi_init();
    if (wifi_err != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi initialization failed: %s", esp_err_to_name(wifi_err));
    }

    esp_err_t bt_err = gba_bt_init();
    if ((bt_err != ESP_OK) && (bt_err != ESP_ERR_NOT_SUPPORTED)) {
        ESP_LOGW(TAG, "Bluetooth initialization failed: %s", esp_err_to_name(bt_err));
    }

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
