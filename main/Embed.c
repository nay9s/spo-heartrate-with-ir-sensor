/*
 * main.c — Heart Rate + SpO2 Monitor
 *
 * Flow:
 *   วางนิ้ว → debounce 600ms
 *   → COUNTDOWN  3 วิ  (3..2..1 อัพเดตทีละวินาที)
 *   → WARMUP    10 วิ  (pipeline + นับ BPM แต่ไม่แสดง)
 *   → MEASURING 30 วิ  (แสดง BPM live)
 *   → RESULT           (สรุปเฉลี่ย + LINE Notify)
 *
 *   ถอดนิ้วทุก state → IDLE ทันที (รีเซ็ต finger state ด้วย)
 *
 * BPM display logic:
 *   - ใช้ hysteresis deadband ±2 BPM กัน BPM กระโดดไปมา
 *   - force refresh ทุก 5 วิ แม้ BPM ไม่เปลี่ยน
 *   - OLED ไม่ clear_screen ทั้งหน้า — เขียนทับเฉพาะบรรทัดที่เปลี่ยน
 *
 * Motion Fix v2: Spike filter + 3-grade adaptive motion + Stability gate
 */

#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "ssd1306.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_netif.h"

// ── Hardware ──────────────────────────────────────────────────────
#define SDA_GPIO        21
#define SCL_GPIO        22
#define I2C_MASTER_NUM  I2C_NUM_0
#define MAX30102_ADDR   0x57
#define REG_FIFO_DATA   0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA     0x0C
#define REG_LED2_PA     0x0D
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define LED_PA          0x24

// ── Timing @ 100 SPS ─────────────────────────────────────────────
#define SAMPLE_RATE          100
#define COUNTDOWN_SEC        3
#define WARMUP_SAMPLES       1000    // 10 วิ — นับ BPM แต่ไม่แสดง
#define MEASURE_SAMPLES      3000    // 30 วิ — วัดจริง
#define TRANSITION_SAMPLES   200
#define MIN_BEATS            4

// ── BPM pipeline ─────────────────────────────────────────────────
#define REFRACTORY_MS        450

// ── BPM display — hysteresis + force refresh ─────────────────────
#define BPM_DEADBAND          2      // ±2 BPM ถือว่าเท่าเดิม
#define BPM_FORCE_REFRESH_MS  5000   // force refresh ทุก 5 วิ

// ── Finger detect ────────────────────────────────────────────────
#define FINGER_ON_THRESHOLD  10000
#define FINGER_OFF_THRESHOLD  5000
#define FINGER_DEBOUNCE_MS    600

// ── SpO2 filter ──────────────────────────────────────────────────

// ── Motion Fix v2 ────────────────────────────────────────────────
#define SPIKE_THRESHOLD           30000
#define MOTION_SOFT_FACTOR         3.0f
#define MOTION_HARD_FACTOR         8.0f
#define MOTION_EXTREME_FACTOR     20.0f
#define EXTREME_IR_JUMP           50000
#define MOTION_SOFT_LOCKOUT_MS     800
#define MOTION_HARD_LOCKOUT_MS    2000
#define MOTION_EXTREME_LOCKOUT_MS 3000
#define MOTION_SOFT_CONSEC         3
#define MOTION_HARD_CONSEC         2
#define MOTION_EXTREME_CONSEC      1
#define STABILITY_WINDOW           20
#define STABILITY_THRESHOLD        0.5f

// ── BPM moving-average ────────────────────────────────────────────
#define BPM_BUF 8

// ── SpO2 ─────────────────────────────────────────────────────────
#define SPO2_A          104.0f
#define SPO2_B           12.0f
#define SPO2_MIN_VALID   80.0f
#define SPO2_MAX_VALID  100.0f
#define SPO2_R_MIN        0.3f
#define SPO2_R_MAX        1.4f
#define SPO2_R_SLOTS       17

// ── Wi-Fi & LINE ─────────────────────────────────────────────────
#define WIFI_SSID  "Suriya Home_2.4GHz"
#define WIFI_PASS  "0856254992"
#define LINE_TOKEN "+I8fJgVuTpjZvtmxd0c3CgtJpSbAu6AAmqBteZnD9YvsvpvhOXWSBPyaUiHNROEdvcfy3QOBR2g1s/2YByftAxOtDE9HwyEOb2HAz7YlK2Tg6Ean/KowS/uAUl2jGaQNmp/7d+9tyxZ3ADoZ2CDEVgdB04t89/1O/w1cDnyilFU="

static const char *TAG = "HR";
static SSD1306_t   dev;

// ── State machine ─────────────────────────────────────────────────
typedef enum {
    STATE_IDLE,
    STATE_WAIT_FINGER,
    STATE_COUNTDOWN,
    STATE_WARMUP,
    STATE_MEASURING,
    STATE_RESULT
} hr_state_t;
static hr_state_t state = STATE_IDLE;

// ── BPM signal variables ──────────────────────────────────────────
static float   dc        = 0.0f;
static float   filtered  = 0.0f;
static float   prev      = 0.0f;
static float   prev2     = 0.0f;
static float   amplitude = 0.0f;
static float   threshold = 0.0f;
static float   bpm       = 0.0f;
static float   bpm_avg   = 0.0f;
static int64_t last_beat = 0;

static float bpm_buf[BPM_BUF];
static int   bpm_i = 0;

// ── SpO2 filter variables ────────────────────────────────────────
static float dc_ir, lp_ir, dc_red, lp_red;

// ── State tracking ───────────────────────────────────────────────
static int      sample_count    = 0;
static double   spo2_sum_ac_red, spo2_sum_dc_red;
static double   spo2_sum_ac_ir,  spo2_sum_dc_ir;
static int      spo2_n          = 0;
static float    spo2_r_slots[SPO2_R_SLOTS];
static int      spo2_r_slot_idx = 0;
static int      motion_total    = 0;
static bool     wifi_connected  = false;
static uint32_t finger_first_seen_ms = 0;
static uint32_t state_enter_ms       = 0;

static int   beat_count   = 0;
static int   beat_counter = 0;
static float beat_pulse   = 0.0f;

// ── BPM display state ─────────────────────────────────────────────
/*
 * bpm_display : BPM ที่แสดงบน OLED ตอนนี้ (หลัง hysteresis)
 * last_refresh_ms : เวลาที่ update OLED ล่าสุด
 */
static float    bpm_display     = 0.0f;
static uint32_t last_refresh_ms = 0;

// ── OLED line cache — เขียนทับเฉพาะ row ที่เปลี่ยน (กัน flicker) ──
#define OLED_ROWS 8
static char oled_cache[OLED_ROWS][17];   // 16 chars + null
static bool oled_full_draw = true;        // true = ต้อง draw ทั้งหน้าครั้งแรก

static void oled_put_row(int row, const char *text16, bool invert)
{
    if (!oled_full_draw && memcmp(oled_cache[row], text16, 16) == 0)
        return;   /* ไม่มีการเปลี่ยน — ข้ามไป */
    memcpy(oled_cache[row], text16, 16);
    oled_cache[row][16] = '\0';
    ssd1306_display_text(&dev, row, text16, 16, invert);
}

static void oled_invalidate(void)
{
    oled_full_draw = true;
    memset(oled_cache, 0, sizeof(oled_cache));
    ssd1306_clear_screen(&dev, false);
}

static void oled_commit(void)
{
    oled_full_draw = false;
}

// ── Motion state ───────────────────────────────────────────────
typedef enum {
    MOTION_NONE = 0,
    MOTION_SOFT = 1,
    MOTION_HARD = 2,
    MOTION_EXTREME = 3
} motion_grade_t;

static motion_grade_t motion_grade            = MOTION_NONE;
static uint32_t       motion_lockout_until_ms = 0;
static int            motion_soft_consec      = 0;
static int            motion_hard_consec      = 0;
static int            motion_extreme_consec   = 0;

static float    stab_buf[STABILITY_WINDOW];
static int      stab_idx  = 0;
static bool     stab_full = false;
static bool     is_stable = true;

static uint32_t prev_raw_ir   = 0;
static float    bpm_avg_saved = 0.0f;

static int last_countdown_sec = -1;

typedef struct { float bpm; float spo2; int beats; int motions; } line_task_arg_t;

// ── Finger state ───────────────────────────────────────────────
static bool finger_state = false;

static void finger_reset(void)     { finger_state = false; }
static bool is_finger_on(uint32_t ir)
{
    if (!finger_state && ir >= FINGER_ON_THRESHOLD)  finger_state = true;
    if ( finger_state && ir <  FINGER_OFF_THRESHOLD) finger_state = false;
    return finger_state;
}

// ─────────────────────────────────────────────────────────────────
//  Wi-Fi
// ─────────────────────────────────────────────────────────────────
static void wifi_event_handler(void *a, esp_event_base_t base,
                                int32_t id, void *d)
{
    if      (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        { wifi_connected = false; esp_wifi_connect(); }
    else if (base == IP_EVENT   && id == IP_EVENT_STA_GOT_IP)
        wifi_connected = true;
}

static void connect_wifi(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));
    wifi_config_t wc = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ─────────────────────────────────────────────────────────────────
//  LINE Notify
// ─────────────────────────────────────────────────────────────────
static void send_line_notify(float bpm_r, float spo2, int beats, int motions)
{
    /* ── BPM line + warning ──────────────────────────────────── */
    char bpm_line[64];
    if (bpm_r <= 0)
        snprintf(bpm_line, sizeof(bpm_line), "BPM   : --");
    else if (bpm_r > 140)
        snprintf(bpm_line, sizeof(bpm_line), "BPM   : %.0f  [!!Very Fast]", bpm_r);
    else if (bpm_r > 120)
        snprintf(bpm_line, sizeof(bpm_line), "BPM   : %.0f  [!Fast]", bpm_r);
    else if (bpm_r < 40)
        snprintf(bpm_line, sizeof(bpm_line), "BPM   : %.0f  [!!Very Slow]", bpm_r);
    else if (bpm_r < 50)
        snprintf(bpm_line, sizeof(bpm_line), "BPM   : %.0f  [!Slow]", bpm_r);
    else
        snprintf(bpm_line, sizeof(bpm_line), "BPM   : %.0f  [Normal]", bpm_r);

    /* ── SpO2 line + warning ─────────────────────────────────── */
    char spo2_line[64];
    if (spo2 < SPO2_MIN_VALID || spo2 > SPO2_MAX_VALID)
        snprintf(spo2_line, sizeof(spo2_line), "SpO2  : --");
    else if (spo2 >= 95.0f)
        snprintf(spo2_line, sizeof(spo2_line), "SpO2  : %.1f%%  [Normal]", spo2);
    else if (spo2 >= 90.0f)
        snprintf(spo2_line, sizeof(spo2_line), "SpO2  : %.1f%%  [!Low]", spo2);
    else
        snprintf(spo2_line, sizeof(spo2_line), "SpO2  : %.1f%%  [!!Danger]", spo2);

    /* ── Beats + Motion ──────────────────────────────────────── */
    char beat_line[32], mot_line[32];
    snprintf(beat_line, sizeof(beat_line), "Beats : %d", beats);
    if (motions > 0)
        snprintf(mot_line, sizeof(mot_line), "Moved : %d times", motions);
    else
        snprintf(mot_line, sizeof(mot_line), "Moved : none");

    /* ── Summary warning (บรรทัดสุดท้าย ถ้ามี) ─────────────── */
    char warn[128] = "";
    if      (bpm_r > 140)
        snprintf(warn, sizeof(warn), ">> Heart rate VERY FAST. See doctor!");
    else if (bpm_r > 120)
        snprintf(warn, sizeof(warn), ">> Heart rate high. Take a rest.");
    else if (bpm_r > 0 && bpm_r < 40)
        snprintf(warn, sizeof(warn), ">> Heart rate VERY SLOW. See doctor!");
    else if (bpm_r > 0 && bpm_r < 50)
        snprintf(warn, sizeof(warn), ">> Heart rate low. Check condition.");

    if (spo2 >= SPO2_MIN_VALID && spo2 < 90.0f) {
        if (warn[0] == '\0')
            snprintf(warn, sizeof(warn), ">> SpO2 CRITICALLY LOW. See doctor!");
        else
            strncat(warn, " / SpO2 critically low!",
                    sizeof(warn) - strlen(warn) - 1);
    } else if (spo2 >= 90.0f && spo2 < 95.0f && warn[0] == '\0') {
        snprintf(warn, sizeof(warn), ">> SpO2 below normal. Monitor closely.");
    }

    /* ── Build JSON body ─────────────────────────────────────── */
    char body[700];
    if (warn[0] != '\0') {
        snprintf(body, sizeof(body),
                 "{\"messages\":[{\"type\":\"text\",\"text\":"
                 "\"== RESULT ==\\n%s\\n%s\\n%s\\n%s\\n"
                 "------------\\n%s\"}]}",
                 bpm_line, spo2_line, beat_line, mot_line, warn);
    } else {
        snprintf(body, sizeof(body),
                 "{\"messages\":[{\"type\":\"text\",\"text\":"
                 "\"== RESULT ==\\n%s\\n%s\\n%s\\n%s\"}]}",
                 bpm_line, spo2_line, beat_line, mot_line);
    }

    esp_http_client_config_t cfg = {
        .url               = "https://api.line.me/v2/bot/message/broadcast",
        .method            = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = 5000,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    char auth[300];
    snprintf(auth, sizeof(auth), "Bearer %s", LINE_TOKEN);
    esp_http_client_set_header(c, "Content-Type",  "application/json");
    esp_http_client_set_header(c, "Authorization", auth);
    esp_http_client_set_post_field(c, body, strlen(body));
    esp_err_t err = esp_http_client_perform(c);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "LINE status=%d", esp_http_client_get_status_code(c));
    else
        ESP_LOGE(TAG, "LINE err=%s", esp_err_to_name(err));
    esp_http_client_cleanup(c);
}

static void __attribute__((unused)) line_task(void *arg)
{
    line_task_arg_t *a = arg;
    send_line_notify(a->bpm, a->spo2, a->beats, a->motions);
    free(a);
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────────────────────────
//  MAX30102 — I2C
// ─────────────────────────────────────────────────────────────────
static void max_write(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

static void max_read_sample(uint32_t *red, uint32_t *ir)
{
    uint8_t d[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, REG_FIFO_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, d, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    *red = (((uint32_t)d[0]<<16)|((uint32_t)d[1]<<8)|d[2]) & 0x3FFFF;
    *ir  = (((uint32_t)d[3]<<16)|((uint32_t)d[4]<<8)|d[5]) & 0x3FFFF;
}


static void max30102_init(void)
{
    max_write(REG_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100));
    max_write(REG_FIFO_WR_PTR, 0);
    max_write(REG_OVF_COUNTER, 0);
    max_write(REG_FIFO_RD_PTR, 0);
    max_write(REG_FIFO_CONFIG, 0x0F);
    max_write(REG_MODE_CONFIG, 0x03);
    max_write(REG_SPO2_CONFIG, 0x27);
    max_write(REG_LED1_PA, LED_PA);
    max_write(REG_LED2_PA, LED_PA);
}

// ─────────────────────────────────────────────────────────────────
//  Reset helpers
// ─────────────────────────────────────────────────────────────────
static void reset_filters(uint32_t ir_seed, uint32_t red_seed)
{
    dc_ir  = (float)ir_seed;
    dc_red = (float)red_seed;
    lp_ir  = lp_red = 0.0f;
    sample_count    = 0;
    spo2_sum_ac_red = spo2_sum_dc_red = spo2_sum_ac_ir = spo2_sum_dc_ir = 0.0;
    spo2_n          = spo2_r_slot_idx = 0;
    for (int i = 0; i < SPO2_R_SLOTS; i++) spo2_r_slots[i] = -1.0f;
    motion_total            = 0;
    motion_grade            = MOTION_NONE;
    motion_lockout_until_ms = 0;
    motion_soft_consec      = 0;
    motion_hard_consec      = 0;
    motion_extreme_consec   = 0;
    stab_idx  = 0;
    stab_full = false;
    is_stable = true;
    prev_raw_ir = ir_seed;
}

static void reset_bpm_pipeline(void)
{
    filtered      = 0.0f;
    prev          = 0.0f;
    prev2         = 0.0f;
    amplitude     = 0.0f;
    threshold     = 0.0f;
    bpm           = 0.0f;
    bpm_avg       = 0.0f;
    bpm_avg_saved = 0.0f;
    bpm_display   = 0.0f;
    last_beat     = 0;
    bpm_i         = 0;
    beat_count    = 0;
    beat_counter  = 0;
    beat_pulse    = 0.0f;
    for (int i = 0; i < BPM_BUF; i++) bpm_buf[i] = 0.0f;
    last_refresh_ms = 0;
}

static void soft_reset_bpm_pipeline(void)
{
    bpm_avg_saved = bpm_avg;
    prev      = 0.0f;
    prev2     = 0.0f;
    last_beat = 0;
    beat_pulse = 0.0f;
    stab_idx  = 0;
    stab_full = false;
    is_stable = false;
}

// ─────────────────────────────────────────────────────────────────
//  Motion Fix v2 — Spike filter
// ─────────────────────────────────────────────────────────────────
static uint32_t spike_filter_ir(uint32_t raw_ir)
{
    uint32_t result = raw_ir;
    int32_t  jump   = (int32_t)raw_ir - (int32_t)prev_raw_ir;
    if (jump < 0) jump = -jump;
    if (jump > SPIKE_THRESHOLD && prev_raw_ir > 0)
        result = prev_raw_ir;
    else
        prev_raw_ir = raw_ir;
    return result;
}

// ─────────────────────────────────────────────────────────────────
//  Motion Fix v2 — Stability gate
// ─────────────────────────────────────────────────────────────────
static bool update_stability(float sig)
{
    stab_buf[stab_idx] = sig;
    stab_idx = (stab_idx + 1) % STABILITY_WINDOW;
    if (stab_idx == 0) stab_full = true;
    if (!stab_full) return false;
    float sum = 0.0f;
    for (int i = 0; i < STABILITY_WINDOW; i++) sum += stab_buf[i];
    float mean = sum / STABILITY_WINDOW;
    float var  = 0.0f;
    for (int i = 0; i < STABILITY_WINDOW; i++) {
        float d = stab_buf[i] - mean; var += d * d;
    }
    var /= STABILITY_WINDOW;
    float amp2 = amplitude * amplitude;
    float nv   = (amp2 > 1.0f) ? (var / amp2) : var;
    return (nv < STABILITY_THRESHOLD);
}

// ─────────────────────────────────────────────────────────────────
//  Motion Fix v2 — 3-grade adaptive detector
// ─────────────────────────────────────────────────────────────────
static void check_motion_v2(float sig, uint32_t raw_ir, uint32_t now_ms)
{
    if (amplitude < 10.0f) return;
    float abs_sig     = fabsf(sig);
    float thr_soft    = amplitude * MOTION_SOFT_FACTOR;
    float thr_hard    = amplitude * MOTION_HARD_FACTOR;
    float thr_extreme = amplitude * MOTION_EXTREME_FACTOR;
    int32_t ir_jump = (int32_t)raw_ir - (int32_t)prev_raw_ir;
    if (ir_jump < 0) ir_jump = -ir_jump;

    if (abs_sig > thr_extreme || ir_jump > EXTREME_IR_JUMP) {
        if (++motion_extreme_consec >= MOTION_EXTREME_CONSEC) {
            motion_extreme_consec = motion_hard_consec = motion_soft_consec = 0;
            motion_total++;
            motion_grade            = MOTION_EXTREME;
            motion_lockout_until_ms = now_ms + MOTION_EXTREME_LOCKOUT_MS;
            dc = (float)raw_ir; reset_bpm_pipeline();
            dc_ir = (float)raw_ir; lp_ir = lp_red = 0.0f;
            stab_idx = 0; stab_full = false; is_stable = false;
            ESP_LOGW(TAG, "[MOTION EXTREME] #%d", motion_total);
        }
        return;
    } else { motion_extreme_consec = 0; }

    if (abs_sig > thr_hard) {
        if (++motion_hard_consec >= MOTION_HARD_CONSEC) {
            motion_hard_consec = motion_soft_consec = 0;
            motion_total++;
            motion_grade            = MOTION_HARD;
            motion_lockout_until_ms = now_ms + MOTION_HARD_LOCKOUT_MS;
            soft_reset_bpm_pipeline();
            dc_ir = (float)raw_ir; lp_ir = lp_red = 0.0f;
            ESP_LOGW(TAG, "[MOTION HARD] #%d", motion_total);
        }
        return;
    } else { motion_hard_consec = 0; }

    if (abs_sig > thr_soft) {
        if (++motion_soft_consec >= MOTION_SOFT_CONSEC) {
            motion_soft_consec = 0;
            if (motion_grade < MOTION_SOFT) { motion_grade = MOTION_SOFT; motion_total++; }
            uint32_t nu = now_ms + MOTION_SOFT_LOCKOUT_MS;
            if (nu > motion_lockout_until_ms) motion_lockout_until_ms = nu;
        }
        return;
    } else { motion_soft_consec = 0; }

    if (motion_grade != MOTION_NONE && now_ms >= motion_lockout_until_ms)
        motion_grade = MOTION_NONE;
}

// ─────────────────────────────────────────────────────────────────
//  BPM moving-average
// ─────────────────────────────────────────────────────────────────
static float bpm_filter_fn(float v)
{
    if (beat_count == 0) {
        for (int i = 0; i < BPM_BUF; i++) bpm_buf[i] = v;
        bpm_i = 0;
    }
    bpm_buf[bpm_i] = v;
    bpm_i = (bpm_i + 1) % BPM_BUF;
    float sum = 0.0f;
    for (int i = 0; i < BPM_BUF; i++) sum += bpm_buf[i];
    return sum / BPM_BUF;
}

// ─────────────────────────────────────────────────────────────────
//  Peak / beat detection
// ─────────────────────────────────────────────────────────────────
static bool detect_peak(float s, uint32_t now_ms)
{
    bool beat = false;
    int64_t now = esp_timer_get_time() / 1000;
    bool stable_now = update_stability(s);
    if (!is_stable && stable_now) { is_stable = true; ESP_LOGI(TAG, "Signal stable"); }

    if (prev2 < prev && prev > s && prev > threshold) {
        if (now - last_beat > REFRACTORY_MS) {
            bool in_lockout = (now_ms < motion_lockout_until_ms);
            if (!in_lockout && is_stable) {
                beat = true;
                if (last_beat > 0) {
                    float dt = (now - last_beat) / 1000.0f;
                    float nb = 60.0f / dt;
                    if (nb > 40 && nb < 180) {
                        bpm     = nb;
                        bpm_avg = bpm_filter_fn(bpm);
                        beat_count++;
                    }
                }
                last_beat = now; beat_counter++; beat_pulse = 1.0f;
                ESP_LOGI(TAG, "BEAT bpm=%.1f avg=%.1f #%d", bpm, bpm_avg, beat_counter);
            } else if (in_lockout && bpm_avg_saved > 0.0f && bpm_avg < 1.0f) {
                bpm_avg = bpm_avg_saved;
            }
        }
    }
    prev2 = prev; prev = s;
    return beat;
}

// ─────────────────────────────────────────────────────────────────
//  SpO2 calculation
// ─────────────────────────────────────────────────────────────────
static float calc_spo2(void)
{
    if (spo2_n < 100) return -1.0f;
    float vr[SPO2_R_SLOTS]; int nv = 0;
    for (int i = 0; i < spo2_r_slot_idx; i++)
        if (spo2_r_slots[i] >= SPO2_R_MIN && spo2_r_slots[i] <= SPO2_R_MAX)
            vr[nv++] = spo2_r_slots[i];
    float R;
    if (nv >= 3) {
        for (int i=0;i<nv-1;i++)
            for (int j=0;j<nv-1-i;j++)
                if (vr[j]>vr[j+1]){float t=vr[j];vr[j]=vr[j+1];vr[j+1]=t;}
        float med=vr[nv/2],sum=0;int cnt=0;
        for (int i=0;i<nv;i++)
            if (fabsf(vr[i]-med)/med<=0.20f){sum+=vr[i];cnt++;}
        R = cnt?sum/cnt:med;
    } else {
        double ra=sqrt(spo2_sum_ac_red/spo2_n),ia=sqrt(spo2_sum_ac_ir/spo2_n);
        double rd=spo2_sum_dc_red/spo2_n,id=spo2_sum_dc_ir/spo2_n;
        if (rd<100||id<100||ia<0.1) return -1.0f;
        R=(float)((ra/rd)/(ia/id));
        if (R<SPO2_R_MIN||R>SPO2_R_MAX) return -1.0f;
    }
    float spo2=SPO2_A-SPO2_B*R;
    ESP_LOGI(TAG,"SpO2 R=%.4f->%.1f%% slots=%d",R,spo2,nv);
    return (spo2>=SPO2_MIN_VALID&&spo2<=SPO2_MAX_VALID)?spo2:-1.0f;
}

// ─────────────────────────────────────────────────────────────────
//  BPM display — hysteresis ±BPM_DEADBAND + force refresh ทุก 5 วิ
//  คืน true เมื่อควร refresh OLED
// ─────────────────────────────────────────────────────────────────
static bool bpm_hysteresis_update(float raw_avg, uint32_t now_ms)
{
    bool val_changed = false;

    if (raw_avg > 1.0f) {
        if (bpm_display < 1.0f) {
            /* ครั้งแรก — ตั้งค่าทันทีไม่ต้องรอ deadband */
            bpm_display = raw_avg;
            val_changed = true;
        } else {
            float diff = raw_avg - bpm_display;
            if (diff < 0) diff = -diff;
            if (diff > (float)BPM_DEADBAND) {
                bpm_display = raw_avg;
                val_changed = true;
            }
        }
    }

    bool force = (now_ms - last_refresh_ms) >= BPM_FORCE_REFRESH_MS;
    return (val_changed || force);
}

// ─────────────────────────────────────────────────────────────────
//  OLED helpers
// ─────────────────────────────────────────────────────────────────

/* สร้าง progress bar 16 chars: "[####--------]  " */
static void make_progress_bar(char out[17], int val, int total)
{
    int pct  = (val * 100) / (total > 0 ? total : 1);
    if (pct > 100) pct = 100;
    int bars = (pct * 12) / 100;
    out[0] = '[';
    for (int i = 0; i < 12; i++) out[1+i] = (i < bars) ? '#' : '-';
    out[13] = ']'; out[14] = ' '; out[15] = ' '; out[16] = '\0';
}

/* สร้าง motion status row 16 chars */
static void make_motion_row(char out[17])
{
    int  m  = motion_total > 99 ? 99 : motion_total;
    char d0 = (char)('0' + m / 10);
    char d1 = (char)('0' + m % 10);
    if (motion_grade == MOTION_EXTREME) {
        memcpy(out, "!!MOTION!! XX   ", 17); out[11]=d0; out[12]=d1;
    } else if (motion_grade == MOTION_HARD) {
        memcpy(out, "!Motion!  XX    ", 17); out[10]=d0; out[11]=d1;
    } else if (motion_grade == MOTION_SOFT) {
        memcpy(out, "  ~Moving~      ", 17);
    } else if (motion_total > 0) {
        memcpy(out, " Moved: XX times", 17); out[8]=d0; out[9]=d1;
    } else {
        memcpy(out, "                ", 17);
    }
    out[16] = '\0';
}

// ─────────────────────────────────────────────────────────────────
//  OLED screens
// ─────────────────────────────────────────────────────────────────

static void screen_idle(void)
{
    oled_invalidate();
    oled_put_row(0, " Heart Rate Mon ", false);
    oled_put_row(1, "                ", false);
    oled_put_row(2, "  Place finger  ", false);
    oled_put_row(3, "   on sensor    ", false);
    oled_put_row(4, "                ", false);
    oled_put_row(5, "  HR + SpO2     ", false);
    oled_put_row(6, "  measurement   ", false);
    oled_put_row(7, "                ", false);
    oled_commit();
}

static void screen_countdown(int sec_left)
{
    char tmp[17];
    /* ครั้งแรก — วาด frame ทั้งหน้า */
    if (oled_full_draw) {
        oled_invalidate();
        oled_put_row(0, "  Get Ready...  ", false);
        oled_put_row(1, "                ", false);
        oled_put_row(2, "  Keep finger   ", false);
        oled_put_row(3, "  still please  ", false);
        oled_put_row(5, "   seconds...   ", false);
        oled_put_row(6, "                ", false);
        oled_put_row(7, "                ", false);
        oled_commit();
    }
    /* อัพเดตเฉพาะบรรทัด 4 (ตัวเลข) */
    memcpy(tmp, "       X        ", 17);
    tmp[7] = (sec_left > 0) ? (char)('0' + (sec_left % 10)) : '!';
    tmp[16] = '\0';
    oled_put_row(4, tmp, false);
}

static void screen_warmup(int sample)
{
    char tmp[17];
    if (oled_full_draw) {
        oled_invalidate();
        oled_put_row(0, "  Calibrating   ", false);
        oled_put_row(1, "                ", false);
        oled_put_row(2, "  Analyzing     ", false);
        oled_put_row(3, "  pulse signal  ", false);
        oled_put_row(4, "                ", false);
        oled_commit();
    }
    /* progress bar — row 5 */
    char bar[17]; make_progress_bar(bar, sample, WARMUP_SAMPLES);
    oled_put_row(5, bar, false);

    /* เวลา — row 6 */
    int sec = sample / SAMPLE_RATE;
    memcpy(tmp, "   XXs / 10s    ", 17);
    tmp[3] = '0'+(char)(sec/10);
    tmp[4] = '0'+(char)(sec%10);
    tmp[16] = '\0';
    oled_put_row(6, tmp, false);
    oled_put_row(7, "                ", false);
}

/*
 * screen_measuring_init — วาด frame คงที่ทั้งหน้าครั้งเดียว
 * เรียกตอนเข้า STATE_MEASURING
 */
static void screen_measuring_init(void)
{
    oled_invalidate();
    oled_put_row(0, "  Measuring     ", true);
    oled_put_row(1, "[------------]  ", false);
    oled_put_row(2, "                ", false);
    oled_put_row(3, "   --- BPM      ", false);
    oled_put_row(4, "  Beats:  000   ", false);
    oled_put_row(5, "  SpO2: calc... ", false);
    oled_put_row(6, "                ", false);
    oled_put_row(7, "                ", false);
    oled_commit();
}

/*
 * screen_measuring_update — เขียนทับเฉพาะ row ที่เปลี่ยน
 * ไม่ clear_screen → จอไม่กระพริบ
 */
static void screen_measuring_update(int sample, int beats, float live_bpm)
{
    char tmp[17];

    /* row 0 : เวลาที่เหลือ */
    int sec_left = ((MEASURE_SAMPLES - sample) + (SAMPLE_RATE-1)) / SAMPLE_RATE;
    if (sec_left < 0) sec_left = 0;
    int sl = sec_left > 99 ? 99 : sec_left;
    memcpy(tmp, "  Measuring XXs ", 17);
    tmp[12] = '0'+(char)(sl/10);
    tmp[13] = '0'+(char)(sl%10);
    tmp[16] = '\0';
    oled_put_row(0, tmp, true);

    /* row 1 : progress bar */
    char bar[17]; make_progress_bar(bar, sample, MEASURE_SAMPLES);
    oled_put_row(1, bar, false);

    /* row 3 : BPM */
    int ibpm = (live_bpm > 0) ? (int)(live_bpm + 0.5f) : 0;
    if (ibpm > 0) {
        memcpy(tmp, "   XXX BPM      ", 17);
        tmp[3] = (ibpm>99)?(char)('0'+ibpm/100):' ';
        tmp[4] = '0'+(char)((ibpm/10)%10);
        tmp[5] = '0'+(char)(ibpm%10);
    } else {
        memcpy(tmp, "   --- BPM      ", 17);
    }
    tmp[16] = '\0';
    oled_put_row(3, tmp, false);

    /* row 4 : beats */
    int b = beats > 999 ? 999 : beats;
    memcpy(tmp, "  Beats:  XXX   ", 17);
    tmp[10] = '0'+(char)(b/100);
    tmp[11] = '0'+(char)((b/10)%10);
    tmp[12] = '0'+(char)(b%10);
    tmp[16] = '\0';
    oled_put_row(4, tmp, false);

    /* row 6 : motion status */
    { char mot[17]; make_motion_row(mot); oled_put_row(6, mot, false); }
}

static void screen_result(float bpm_r, float spo2, int motions)
{
    char tmp[17];
    oled_invalidate();
    oled_put_row(0, "    Result      ", true);
    oled_put_row(1, "                ", false);

    if (bpm_r > 0) {
        int b=(int)(bpm_r+0.5f);
        memcpy(tmp,"  HR:  XXX BPM  ",17);
        tmp[7]=(b>99)?(char)('0'+b/100):' ';
        tmp[8]='0'+(char)((b/10)%10);
        tmp[9]='0'+(char)(b%10);
    } else { memcpy(tmp,"  HR:   -- BPM  ",17); }
    tmp[16]='\0'; oled_put_row(2, tmp, false);

    if (spo2>=SPO2_MIN_VALID&&spo2<=SPO2_MAX_VALID) {
        int si=(int)spo2, sf=(int)((spo2-(float)si)*10.0f+0.5f);
        memcpy(tmp,"  SpO2:  XX.X%  ",17);
        tmp[9]='0'+(char)(si/10); tmp[10]='0'+(char)(si%10);
        tmp[12]='0'+(char)(sf%10);
    } else { memcpy(tmp,"  SpO2:  -- %   ",17); }
    tmp[16]='\0'; oled_put_row(3, tmp, false);

    if      (spo2>=95.0f)         oled_put_row(4,"  Status: Normal",false);
    else if (spo2>=90.0f)         oled_put_row(4,"  Status: Low   ",false);
    else if (spo2>=SPO2_MIN_VALID)oled_put_row(4,"  Status: Danger",false);
    else                          oled_put_row(4,"                ",false);

    oled_put_row(5,"                ",false);
    if (motions>0) {
        int m=motions>99?99:motions;
        memcpy(tmp," Motion:  XX evt",17);
        tmp[10]='0'+(char)(m/10); tmp[11]='0'+(char)(m%10);
        tmp[16]='\0'; oled_put_row(6,tmp,false);
    } else { oled_put_row(6,"  Sent to LINE  ",false); }
    oled_put_row(7,"                ",false);
    oled_commit();
}

static void screen_failed(void)
{
    oled_invalidate();
    oled_put_row(0,"  Meas. Failed  ",true);
    oled_put_row(1,"                ",false);
    oled_put_row(2," No pulse found ",false);
    oled_put_row(3,"                ",false);
    oled_put_row(4,"  Keep still &  ",false);
    oled_put_row(5,"  press gently  ",false);
    oled_put_row(6,"  then retry    ",false);
    oled_put_row(7,"                ",false);
    oled_commit();
}

// ─────────────────────────────────────────────────────────────────
//  run_pipeline — BPM + SpO2 + motion + beat (1 sample)
// ─────────────────────────────────────────────────────────────────
static void run_pipeline(uint32_t raw_ir, uint32_t raw_red, uint32_t now_ms)
{
    dc        = 0.97f*dc       + 0.03f*(float)raw_ir;
    float ac  = (float)raw_ir - dc;
    filtered  = 0.92f*filtered + 0.08f*ac;
    amplitude = 0.95f*amplitude+ 0.05f*fabsf(filtered);
    threshold = amplitude * 0.5f;

    dc_ir  = 0.95f*dc_ir  + 0.05f*(float)raw_ir;
    dc_red = 0.95f*dc_red + 0.05f*(float)raw_red;
    lp_ir  = 0.85f*lp_ir  + 0.15f*((float)raw_ir  - dc_ir);
    lp_red = 0.85f*lp_red + 0.15f*((float)raw_red - dc_red);

    if (sample_count >= TRANSITION_SAMPLES) {
        spo2_sum_ac_red += (double)lp_red*lp_red;
        spo2_sum_dc_red += (double)dc_red;
        spo2_sum_ac_ir  += (double)lp_ir*lp_ir;
        spo2_sum_dc_ir  += (double)dc_ir;
        spo2_n++;
        if (spo2_n%100==0 && spo2_r_slot_idx<SPO2_R_SLOTS) {
            double ra=sqrt(spo2_sum_ac_red/spo2_n),ia=sqrt(spo2_sum_ac_ir/spo2_n);
            double rd=spo2_sum_dc_red/spo2_n,id=spo2_sum_dc_ir/spo2_n;
            if (rd>100&&id>100&&ia>0.1)
                spo2_r_slots[spo2_r_slot_idx++]=(float)((ra/rd)/(ia/id));
        }
    }
    check_motion_v2(filtered, raw_ir, now_ms);
    detect_peak(filtered, now_ms);
    sample_count++;
}

/* goto_idle — reset นิ้ว + กลับ STATE_IDLE + วาด screen */
#define goto_idle() do { finger_reset(); state = STATE_IDLE; screen_idle(); } while(0)

// ─────────────────────────────────────────────────────────────────
//  app_main
// ─────────────────────────────────────────────────────────────────
void app_main(void)
{
    connect_wifi();
    while (!wifi_connected) vTaskDelay(500/portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "WiFi ready");

    i2c_master_init(&dev, SDA_GPIO, SCL_GPIO, -1);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    max30102_init();
    finger_reset();
    screen_idle();

    printf(">IR_AC RED_AC LiveBPM Threshold R_ratio BeatPulse BeatCount\n");

    while (1) {
        uint32_t raw_red, raw_ir_raw;
        max_read_sample(&raw_red, &raw_ir_raw);
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        bool     finger = is_finger_on(raw_ir_raw);  /* raw — ไม่ผ่าน spike filter */
        uint32_t raw_ir = finger ? spike_filter_ir(raw_ir_raw) : raw_ir_raw;

        // ════════════════════════════════════════════════════════
        //  STATE_IDLE
        // ════════════════════════════════════════════════════════
        if (state == STATE_IDLE) {
            if (finger) {
                finger_first_seen_ms = now_ms;
                state = STATE_WAIT_FINGER;
            }
        }

        // ════════════════════════════════════════════════════════
        //  STATE_WAIT_FINGER — debounce 600 ms
        // ════════════════════════════════════════════════════════
        else if (state == STATE_WAIT_FINGER) {
            if (!finger) {
                /* ถอดนิ้วระหว่าง debounce → กลับ IDLE */
                goto_idle();
            } else if (now_ms - finger_first_seen_ms >= FINGER_DEBOUNCE_MS) {
                reset_filters(raw_ir, raw_red);
                dc = (float)raw_ir;
                reset_bpm_pipeline();
                prev_raw_ir        = raw_ir;
                state_enter_ms     = now_ms;
                last_countdown_sec = -1;
                state = STATE_COUNTDOWN;
                screen_countdown(COUNTDOWN_SEC);
                ESP_LOGI(TAG, "-> COUNTDOWN");
            }
        }

        // ════════════════════════════════════════════════════════
        //  STATE_COUNTDOWN — 3..2..1
        // ════════════════════════════════════════════════════════
        else if (state == STATE_COUNTDOWN) {
            if (!finger) {
                goto_idle();
                vTaskDelay(10/portTICK_PERIOD_MS);
                continue;
            }
            uint32_t elapsed = now_ms - state_enter_ms;
            int sec_done  = (int)(elapsed / 1000);
            int sec_left  = COUNTDOWN_SEC - sec_done;
            if (sec_left < 0) sec_left = 0;

            if (sec_left != last_countdown_sec) {
                last_countdown_sec = sec_left;
                screen_countdown(sec_left);
            }

            if (elapsed >= (uint32_t)(COUNTDOWN_SEC * 1000)) {
                dc = (float)raw_ir;
                reset_bpm_pipeline();
                sample_count   = 0;
                state_enter_ms = now_ms;
                oled_full_draw = true;   /* บังคับ draw warmup ใหม่ */
                screen_warmup(0);
                state = STATE_WARMUP;
                ESP_LOGI(TAG, "-> WARMUP");
            }
        }

        // ════════════════════════════════════════════════════════
        //  STATE_WARMUP — 10 วิ คำนวณ BPM แต่ไม่แสดง
        // ════════════════════════════════════════════════════════
        else if (state == STATE_WARMUP) {
            if (!finger) {
                goto_idle();
                vTaskDelay(10/portTICK_PERIOD_MS);
                continue;
            }

            run_pipeline(raw_ir, raw_red, now_ms);

            /* อัพเดต progress bar ทุก 0.5 วิ */
            if (sample_count % 50 == 0)
                screen_warmup(sample_count);

            printf("%f %f %f %f %f %f %f\n",
                   filtered/10000.0f, lp_red/10000.0f,
                   bpm_avg/300.0f, threshold/10000.0f,
                   0.0f, beat_pulse, (float)beat_counter/100.0f);
            beat_pulse = 0.0f;

            if (sample_count >= WARMUP_SAMPLES) {
                /* รีเซ็ต beat counter สำหรับ MEASURING
                 * แต่คง bpm_avg ไว้เพื่อแสดงทันที */
                beat_count   = 0;
                beat_counter = 0;
                sample_count = 0;
                bpm_display  = bpm_avg;   /* seed hysteresis */
                last_refresh_ms = now_ms;
                screen_measuring_init();
                screen_measuring_update(0, 0, bpm_display);
                state = STATE_MEASURING;
                ESP_LOGI(TAG, "-> MEASURING bpm=%.1f", bpm_avg);
            }
        }

        // ════════════════════════════════════════════════════════
        //  STATE_MEASURING — 30 วิ แสดง BPM live
        // ════════════════════════════════════════════════════════
        else if (state == STATE_MEASURING) {
            if (!finger) {
                goto_idle();
                vTaskDelay(10/portTICK_PERIOD_MS);
                continue;
            }

            run_pipeline(raw_ir, raw_red, now_ms);

            float raw_avg = (bpm_avg > 0) ? bpm_avg : bpm_avg_saved;

            /* hysteresis + force refresh */
            if (bpm_hysteresis_update(raw_avg, now_ms)) {
                screen_measuring_update(sample_count, beat_count, bpm_display);
                last_refresh_ms = now_ms;
                printf("BPM: %.0f\n", bpm_display);
            }

            /* Serial Plotter */
            float r_live = 0.0f;
            if (spo2_n >= 50) {
                double ra=sqrt(spo2_sum_ac_red/spo2_n),ia=sqrt(spo2_sum_ac_ir/spo2_n);
                double rd=spo2_sum_dc_red/spo2_n,id=spo2_sum_dc_ir/spo2_n;
                if (rd>100&&id>100&&ia>0.1)
                    r_live=(float)((ra/rd)/(ia/id));
            }
            printf("%f %f %f %f %f %f %f\n",
                   filtered/10000.0f, lp_red/10000.0f,
                   bpm_avg/300.0f, threshold/10000.0f,
                   r_live, beat_pulse, (float)beat_counter/100.0f);
            beat_pulse = 0.0f;

            if (sample_count >= MEASURE_SAMPLES)
                state = STATE_RESULT;
        }

        // ════════════════════════════════════════════════════════
        //  STATE_RESULT — สรุปผล + ส่ง LINE
        // ════════════════════════════════════════════════════════
        else if (state == STATE_RESULT) {
            float result_bpm  = (bpm_avg > 0) ? bpm_avg : bpm_avg_saved;
            float result_spo2 = calc_spo2();
            bool  bpm_ok  = (beat_count >= MIN_BEATS && result_bpm > 0);
            bool  spo2_ok = (result_spo2 >= SPO2_MIN_VALID);

            double R_final = -1.0;
            if (spo2_n > 0) {
                double ra=sqrt(spo2_sum_ac_red/spo2_n),ia=sqrt(spo2_sum_ac_ir/spo2_n);
                double rd=spo2_sum_dc_red/spo2_n,id=spo2_sum_dc_ir/spo2_n;
                if (rd>0&&id>0&&ia>0) R_final=(ra/rd)/(ia/id);
            }

            ESP_LOGI(TAG,"==============================");
            ESP_LOGI(TAG,"  RESULT BPM   : %.1f",result_bpm);
            ESP_LOGI(TAG,"  RESULT SpO2  : %.1f%%",result_spo2);
            ESP_LOGI(TAG,"  R-ratio      : %.4f",R_final);
            ESP_LOGI(TAG,"  BEATS (valid): %d",beat_count);
            ESP_LOGI(TAG,"  BEATS (total): %d",beat_counter);
            ESP_LOGI(TAG,"  MOTION       : %d",motion_total);
            ESP_LOGI(TAG,"  SPO2_N       : %d",spo2_n);
            ESP_LOGI(TAG,"==============================");

            if (bpm_ok || spo2_ok) {
                screen_result(bpm_ok?result_bpm:-1.0f, result_spo2, motion_total);
                line_task_arg_t *arg = malloc(sizeof(line_task_arg_t));
                if (arg) {
                    arg->bpm     = bpm_ok  ? result_bpm  : -1.0f;
                    arg->spo2    = spo2_ok ? result_spo2 : -1.0f;
                    arg->beats   = beat_count;
                    arg->motions = motion_total;
                    xTaskCreate(line_task,"line",8192,arg,5,NULL);
                }
            } else {
                screen_failed();
            }

            /* รอถอดนิ้ว */
            finger_reset();
            { uint32_t _r, _ir;
              do { vTaskDelay(100/portTICK_PERIOD_MS);
                   max_read_sample(&_r, &_ir);
              } while (_ir >= FINGER_OFF_THRESHOLD); }
            vTaskDelay(300/portTICK_PERIOD_MS);
            state = STATE_IDLE;
            screen_idle();
        }

        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}