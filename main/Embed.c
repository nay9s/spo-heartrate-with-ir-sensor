#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ssd1306.h"

// ================================================================
//  PIN & I2C
// ================================================================
#define SDA_GPIO       21
#define SCL_GPIO       22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_FREQ       400000

// ================================================================
//  MAX30102
// ================================================================
#define MAX30102_ADDR   0x57
#define REG_FIFO_DATA   0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA     0x0C
#define REG_LED2_PA     0x0D

// ================================================================
//  TIMING @ 100 SPS
// ================================================================
#define SETTLE_SAMPLES      200     // 2 วินาที
#define TRANSITION_SAMPLES  100     // 1 วินาทีแรกของ measuring
#define MEASURE_SAMPLES     2000    // 20 วินาที
#define MIN_BEATS           3       // ลดจาก 6 → แสดงผลเร็วขึ้น

// ================================================================
//  FINGER DETECTION
// ================================================================
#define FINGER_ON_THRESHOLD   20000
#define FINGER_OFF_THRESHOLD  10000
#define FINGER_DEBOUNCE_MS    500

// ================================================================
//  FILTER
//  - LP_ALPHA ลดลงเหลือ 0.10 เพื่อให้สัญญาณเรียบขึ้น
// ================================================================
#define DC_ALPHA_FAST   0.08f   // เดิม 0.10
#define DC_ALPHA_MID    0.03f
#define DC_ALPHA_SLOW   0.005f
#define LP_ALPHA        0.18f   // เดิม 0.35 — สำคัญมาก ลดสไปค์

// ================================================================
//  BEAT DETECTION
//  - REFRACTORY_MS เพิ่มเป็น 600 ป้องกัน double-count
//  - THRESHOLD_RATIO เพิ่มเป็น 0.55 กันจับ noise
//  - THRESHOLD_DECAY ช้าลงเป็น 0.995
//  - MIN_RR_MS = 400 (max ~150 BPM), MAX_RR_MS = 1500 (min ~40 BPM)
// ================================================================
#define REFRACTORY_MS    350    // เดิม 250
#define THRESHOLD_RATIO  0.45f  // เดิม 0.40
#define THRESHOLD_MIN    3.0f   // เดิม 5.0
#define THRESHOLD_DECAY  0.995f // เดิม 0.99
#define MIN_RR_MS        400    // เดิม 250
#define MAX_RR_MS        1500   // เดิม 2000

// ================================================================
//  BPM SMOOTHING — moving average window
// ================================================================
#define BPM_WINDOW  6   // เฉลี่ย 6 beat ล่าสุด

static const char *TAG = "HR";

static SSD1306_t dev;

// ================================================================
//  State machine
// ================================================================
typedef enum {
    STATE_IDLE,
    STATE_WAIT_FINGER,
    STATE_SETTLING,
    STATE_MEASURING,
    STATE_RESULT
} hr_state_t;

hr_state_t state = STATE_IDLE;

float    dc           = 0;
float    lp_signal    = 0;
int      sample_count = 0;

uint32_t last_beat_ms  = 0;
float    beat_bpm[40];
int      beat_count    = 0;

float    peak_val       = 0;
float    auto_threshold = THRESHOLD_MIN;
bool     is_rising      = false;
uint32_t last_peak_ms   = 0;

uint32_t finger_first_seen_ms = 0;

// ================================================================
//  BPM sliding window (ใช้คำนวณ live BPM แบบเรียบ)
// ================================================================
float    bpm_window_buf[BPM_WINDOW];
int      bpm_window_idx = 0;
int      bpm_window_cnt = 0;

//////////////////////////////////////////////////////

static inline uint32_t get_ms()
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

//////////////////////////////////////////////////////
//  MAX30102
//////////////////////////////////////////////////////

void max_write(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

uint32_t max_read_ir()
{
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, REG_FIFO_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    uint32_t ir = ((uint32_t)data[3] << 16) |
                  ((uint32_t)data[4] << 8)  |
                  data[5];
    return ir & 0x3FFFF;
}

void max30102_init()
{
    max_write(REG_MODE_CONFIG, 0x40);   // reset
    vTaskDelay(100 / portTICK_PERIOD_MS);
    max_write(REG_FIFO_CONFIG, 0x4F);   // SMP_AVE=8, FIFO_ROLLOVER=1
    max_write(REG_MODE_CONFIG, 0x03);   // SpO2 mode
    max_write(REG_SPO2_CONFIG, 0x27);   // 100 SPS, 411us pulse
    max_write(REG_LED1_PA,     0x24);   // ~7.2 mA
    max_write(REG_LED2_PA,     0x24);
}

//////////////////////////////////////////////////////
//  Heart rate helpers
//////////////////////////////////////////////////////

void reset_filters(uint32_t ir_seed)
{
    dc        = (float)ir_seed;
    lp_signal = 0;
    sample_count = 0;
}

void reset_beats()
{
    beat_count      = 0;
    last_beat_ms    = 0;
    peak_val        = 0;
    is_rising       = false;
    last_peak_ms    = 0;
    auto_threshold  = THRESHOLD_MIN;
    bpm_window_idx  = 0;
    bpm_window_cnt  = 0;
    for (int i = 0; i < 40; i++) beat_bpm[i] = 0;
    for (int i = 0; i < BPM_WINDOW; i++) bpm_window_buf[i] = 0;
}

bool is_finger_on(uint32_t ir)
{
    static bool fs = false;
    if (!fs && ir >= FINGER_ON_THRESHOLD)  fs = true;
    if ( fs && ir <  FINGER_OFF_THRESHOLD) fs = false;
    return fs;
}

// ------------------------------------------------------------------
//  detect_peak — คืนค่า true เมื่อตรวจพบ 1 heartbeat
//  แก้ไข: refractory ยาวขึ้น, threshold สูงขึ้น, reset peak หลัง beat
// ------------------------------------------------------------------
bool detect_peak(float sig, uint32_t now_ms)
{
    bool beat = false;

    // decay threshold ช้าๆ
    auto_threshold *= THRESHOLD_DECAY;
    if (auto_threshold < THRESHOLD_MIN) auto_threshold = THRESHOLD_MIN;

    if (sig > peak_val)
    {
        peak_val  = sig;
        is_rising = true;
    }
    else if (is_rising && (peak_val - sig) > auto_threshold)
    {
        // ตรวจว่าผ่าน refractory period แล้วหรือยัง
        uint32_t since_last = (last_peak_ms == 0) ? 9999 : (now_ms - last_peak_ms);

        if (since_last >= (uint32_t)REFRACTORY_MS)
        {
            beat = true;

            float nt = peak_val * THRESHOLD_RATIO;
            auto_threshold = (nt > THRESHOLD_MIN) ? nt : THRESHOLD_MIN;

            ESP_LOGI(TAG, "BEAT | peak=%.2f | thresh=%.2f | RR=%lums | beats=%d",
                peak_val, auto_threshold,
                (unsigned long)since_last, beat_count + 1);

            last_peak_ms = now_ms;
        }

        // reset เสมอหลังจากเห็น falling edge
        is_rising = false;
        peak_val  = sig;
    }
    return beat;
}

// ------------------------------------------------------------------
//  calc_live_bpm — เฉลี่ยจาก sliding window ล่าสุด
// ------------------------------------------------------------------
float calc_live_bpm()
{
    if (bpm_window_cnt < 1) return 0;
    float sum = 0;
    for (int i = 0; i < bpm_window_cnt; i++)
        sum += bpm_window_buf[i];
    return sum / bpm_window_cnt;
}

// ------------------------------------------------------------------
//  calc_result_bpm — เฉลี่ยทุก beat ที่อยู่ใน range 30-200 BPM
// ------------------------------------------------------------------
float calc_result_bpm()
{
    if (beat_count < 1) return 0;
    float sum = 0;
    int   n   = 0;
    for (int i = 0; i < beat_count && i < 40; i++)
        if (beat_bpm[i] > 30 && beat_bpm[i] < 200) { sum += beat_bpm[i]; n++; }
    return (n > 0) ? (sum / n) : 0;
}

//////////////////////////////////////////////////////
//  OLED screens (ASCII only, 16 chars per row)
//////////////////////////////////////////////////////

void screen_idle()
{
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, " Heart Rate Mon ", 16, true);
    ssd1306_display_text(&dev, 2, "  Place finger  ", 16, false);
    ssd1306_display_text(&dev, 3, "   on sensor    ", 16, false);
    ssd1306_display_text(&dev, 5, "  to start HR   ", 16, false);
    ssd1306_display_text(&dev, 6, "  measurement   ", 16, false);
}

void screen_settling(int sec_left)
{
    char buf[17];

    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, "  Preparing...  ", 16, true);
    ssd1306_display_text(&dev, 2, "  Keep finger   ", 16, false);
    ssd1306_display_text(&dev, 3, "  still please  ", 16, false);

    if (sec_left > 0)
    {
        memcpy(buf, "   Wait  0s...  ", 17);
        buf[9] = '0' + (char)(sec_left % 10);
    }
    else
    {
        memcpy(buf, "  Starting...   ", 17);
    }

    ssd1306_display_text(&dev, 5, buf, 16, false);
}

void screen_measuring(int sample, int beats, float live_bpm)
{
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, "  Measuring...  ", 16, true);

    // progress bar
    int pct  = (sample * 100) / MEASURE_SAMPLES;
    int bars = (pct * 12) / 100;

    char bar[17];
    bar[0]  = '[';
    for (int i = 0; i < 12; i++)
        bar[1 + i] = (i < bars) ? '#' : '-';
    bar[13] = ']';
    bar[14] = ' ';
    bar[15] = ' ';
    bar[16] = '\0';
    ssd1306_display_text(&dev, 1, bar, 16, false);

    // percent
    char pct_buf[17] = "                ";
    int  p = pct % 1000;
    pct_buf[3]  = '0' + (p / 100);
    pct_buf[4]  = '0' + (p / 10 % 10);
    pct_buf[5]  = '0' + (p % 10);
    pct_buf[6]  = '%';
    pct_buf[16] = '\0';
    ssd1306_display_text(&dev, 2, pct_buf, 16, false);

    // beats
    char beats_buf[17];
    snprintf(beats_buf, sizeof(beats_buf), "  Beats:  %3d   ", beats);
    ssd1306_display_text(&dev, 3, beats_buf, 16, false);

    // live BPM
    if (live_bpm > 0)
    {
        char bpm_str[17];
        snprintf(bpm_str, sizeof(bpm_str), "  Live: %3.0f BPM ", live_bpm);
        ssd1306_display_text(&dev, 5, bpm_str, 16, false);
    }
    else
    {
        ssd1306_display_text(&dev, 5, "  Counting...   ", 16, false);
    }
}

void screen_result(float bpm)
{
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, "    Result      ", 16, true);

    char bpm_str[17];
    snprintf(bpm_str, sizeof(bpm_str), "  Heart: %3.0f BPM", bpm);
    ssd1306_display_text(&dev, 2, bpm_str, 16, false);

    ssd1306_display_text(&dev, 4, "  SpO2:   -- %  ", 16, false);
    ssd1306_display_text(&dev, 7, "Lift->place=redo", 16, true);
}

void screen_failed()
{
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, "  Meas. Failed  ", 16, true);
    ssd1306_display_text(&dev, 2, " No pulse found ", 16, false);
    ssd1306_display_text(&dev, 4, "  Press harder  ", 16, false);
    ssd1306_display_text(&dev, 5, "  then lift &   ", 16, false);
    ssd1306_display_text(&dev, 6, "  place again   ", 16, false);
}

//////////////////////////////////////////////////////
//  app_main
//////////////////////////////////////////////////////

void app_main()
{
    i2c_master_init(&dev, SDA_GPIO, SCL_GPIO, -1);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);

    max30102_init();

    ESP_LOGI(TAG, "Ready");
    // format: signal(scaled) bpm(scaled) state
    // signal / 10000, bpm / 300 เพื่อให้อยู่ใน range เดียวกันบน plotter
    printf("signal bpm state\n");

    screen_idle();

    while (1)
    {
        uint32_t ir     = max_read_ir();
        uint32_t now_ms = get_ms();
        bool     finger = is_finger_on(ir);

        // ================================================
        //  IDLE
        // ================================================
        if (state == STATE_IDLE)
        {
            if (finger)
            {
                finger_first_seen_ms = now_ms;
                ESP_LOGI(TAG, "Finger detected - debouncing...");
                state = STATE_WAIT_FINGER;
            }
            else
            {
                printf("0 0 0\n");
            }
        }

        // ================================================
        //  WAIT_FINGER — debounce 500ms
        // ================================================
        else if (state == STATE_WAIT_FINGER)
        {
            if (!finger)
            {
                ESP_LOGI(TAG, "Finger removed -> IDLE");
                state = STATE_IDLE;
                screen_idle();
                printf("0 0 0\n");
            }
            else if ((now_ms - finger_first_seen_ms) >= FINGER_DEBOUNCE_MS)
            {
                ESP_LOGI(TAG, "Finger confirmed -> SETTLING");
                reset_filters(ir);
                reset_beats();
                state = STATE_SETTLING;
                screen_settling(2);
            }
        }

        // ================================================
        //  SETTLING — 2s
        // ================================================
        else if (state == STATE_SETTLING)
        {
            if (!finger)
            {
                ESP_LOGI(TAG, "Finger removed -> IDLE");
                state = STATE_IDLE;
                screen_idle();
                printf("0 0 0\n");
                vTaskDelay(10 / portTICK_PERIOD_MS);
                continue;
            }

            dc        = dc + ((float)ir - dc) * DC_ALPHA_FAST;
            float ac  = (float)ir - dc;
            lp_signal = lp_signal * (1.0f - LP_ALPHA) + ac * LP_ALPHA;

            sample_count++;

            if (sample_count % 50 == 0)
            {
                int left = (SETTLE_SAMPLES - sample_count) / 100;
                screen_settling(left > 0 ? left : 0);
            }

            if (sample_count % 100 == 0)
                ESP_LOGI(TAG, "Settling %ds left",
                    (SETTLE_SAMPLES - sample_count) / 100);

            printf("%f 0 2\n", lp_signal / 10000.0f);

            if (sample_count >= SETTLE_SAMPLES)
            {
                ESP_LOGI(TAG, "-> MEASURING (20s)");
                reset_beats();
                sample_count   = 0;
                peak_val       = fabsf(lp_signal);
                auto_threshold = THRESHOLD_MIN;
                state          = STATE_MEASURING;
            }
        }

        // ================================================
        //  MEASURING — 20s
        // ================================================
        else if (state == STATE_MEASURING)
        {
            if (!finger)
            {
                ESP_LOGI(TAG, "Finger removed -> IDLE");
                state = STATE_IDLE;
                screen_idle();
                printf("0 0 0\n");
                vTaskDelay(10 / portTICK_PERIOD_MS);
                continue;
            }

            float dc_alpha = (sample_count < TRANSITION_SAMPLES)
                             ? DC_ALPHA_MID : DC_ALPHA_SLOW;

            dc        = dc + ((float)ir - dc) * dc_alpha;
            float ac  = (float)ir - dc;
            lp_signal = lp_signal * (1.0f - LP_ALPHA) + ac * LP_ALPHA;

            if (detect_peak(lp_signal, now_ms))
            {
                uint32_t rr = now_ms - last_beat_ms;
                if (last_beat_ms != 0 && rr >= MIN_RR_MS && rr <= MAX_RR_MS)
                {
                    float instant_bpm = 60000.0f / (float)rr;

                    // เก็บใน array สำหรับ final result
                    if (beat_count < 40)
                        beat_bpm[beat_count++] = instant_bpm;

                    // เก็บใน sliding window สำหรับ live BPM
                    bpm_window_buf[bpm_window_idx] = instant_bpm;
                    bpm_window_idx = (bpm_window_idx + 1) % BPM_WINDOW;
                    if (bpm_window_cnt < BPM_WINDOW) bpm_window_cnt++;
                }
                last_beat_ms = now_ms;
            }

            float live_bpm = calc_live_bpm();

            sample_count++;

            if (sample_count % 20 == 0)
                screen_measuring(sample_count, beat_count, live_bpm);

            if (sample_count % 200 == 0)
                ESP_LOGI(TAG, "Measuring %ds/20s | beats:%d | %.1f BPM",
                    sample_count / 100, beat_count, live_bpm);

            // Serial plotter: scale เพื่อให้ทั้งสองช่องอยู่ใน range ใกล้กัน
            printf("%f %f 3\n", lp_signal / 10000.0f, live_bpm / 300.0f);

            if (sample_count >= MEASURE_SAMPLES)
                state = STATE_RESULT;
        }

        // ================================================
        //  RESULT
        // ================================================
        else if (state == STATE_RESULT)
        {
            float final_bpm = calc_result_bpm();

            ESP_LOGI(TAG, "==============================");
            ESP_LOGI(TAG, "  RESULT : %.1f BPM", final_bpm);
            ESP_LOGI(TAG, "  BEATS  : %d detected", beat_count);
            ESP_LOGI(TAG, "==============================");

            if (beat_count >= MIN_BEATS && final_bpm > 0)
                screen_result(final_bpm);
            else
                screen_failed();

            printf("%f %f 4\n", 0.0f, final_bpm / 300.0f);

            // รอยกนิ้วออก แล้วกลับ IDLE อัตโนมัติ
            ESP_LOGI(TAG, "Waiting for finger lift...");
            while (is_finger_on(max_read_ir()))
                vTaskDelay(100 / portTICK_PERIOD_MS);

            vTaskDelay(300 / portTICK_PERIOD_MS);

            ESP_LOGI(TAG, "-> IDLE");
            state = STATE_IDLE;
            screen_idle();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}