#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
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
#define LED_PA          0x24

// ── Timing @ 100 SPS ─────────────────────────────────────────────
#define SETTLE_SAMPLES      500
#define TRANSITION_SAMPLES  300
#define MEASURE_SAMPLES     2000
#define MIN_BEATS           4

// ── Finger ───────────────────────────────────────────────────────
#define FINGER_ON_THRESHOLD  30000
#define FINGER_OFF_THRESHOLD 15000
#define FINGER_DEBOUNCE_MS   600
#define SAT_THRESHOLD        200000
#define SAT_WARNING_THRESH   170000

// ── Filters ──────────────────────────────────────────────────────
#define DC_ALPHA_FAST  0.10f
#define DC_ALPHA_MID   0.05f
#define DC_ALPHA_SLOW  0.01f
#define LP_ALPHA       0.12f

// ── Beat detection ────────────────────────────────────────────────
#define REFRACTORY_MS    400
#define THRESHOLD_RATIO  0.45f
#define THRESHOLD_MIN    3.0f
#define THRESHOLD_MAX    180.0f
#define THRESHOLD_DECAY  0.995f
#define MIN_RR_MS        450
#define MAX_RR_MS        1500
#define AMP_MIN_MULT     1.5f
#define MOTION_THRESHOLD 1000.0f
#define MOTION_CONSEC    5
#define PEAK_RATIO_MAX   3.5f
#define BPM_WINDOW       6

// ── SpO2 ──────────────────────────────────────────────────────────
#define SPO2_A         104.0f
#define SPO2_B         12.0f
#define SPO2_MIN_VALID 80.0f
#define SPO2_MAX_VALID 100.0f
#define SPO2_R_MIN     0.3f
#define SPO2_R_MAX     1.4f
#define SPO2_R_SLOTS   17

// ── Wi-Fi & LINE ─────────────────────────────────────────────────
#define WIFI_SSID  "Sairung_B16"
#define WIFI_PASS  "690329828"
#define LINE_TOKEN "+I8fJgVuTpjZvtmxd0c3CgtJpSbAu6AAmqBteZnD9YvsvpvhOXWSBPyaUiHNROEdvcfy3QOBR2g1s/2YByftAxOtDE9HwyEOb2HAz7YlK2Tg6Ean/KowS/uAUl2jGaQNmp/7d+9tyxZ3ADoZ2CDEVgdB04t89/1O/w1cDnyilFU="

static const char *TAG = "HR";
static SSD1306_t   dev;

typedef enum { STATE_IDLE, STATE_WAIT_FINGER, STATE_SETTLING, STATE_MEASURING, STATE_RESULT } hr_state_t;
static hr_state_t state = STATE_IDLE;

static float    dc_ir, lp_ir, dc_red, lp_red;
static int      sample_count;

static double   spo2_sum_ac_red, spo2_sum_dc_red, spo2_sum_ac_ir, spo2_sum_dc_ir;
static int      spo2_n;
static float    spo2_r_slots[SPO2_R_SLOTS];
static int      spo2_r_slot_idx;

static uint32_t last_beat_ms, last_peak_ms;
static float    beat_bpm[40], peak_val, auto_threshold, peak_sum;
static int      beat_count, peak_hist_cnt;
static bool     is_rising;
static float    bpm_window_buf[BPM_WINDOW];
static int      bpm_window_idx, bpm_window_cnt;

static int      motion_count, motion_total;
static uint32_t finger_first_seen_ms;
static bool     sat_warned, wifi_connected;

// ── ตัวแปรนับ beat สำหรับ Serial Plotter ─────────────────────────
// beat_pulse   : พุ่ง 1.0 ใน sample ที่ detect peak แล้วกลับ 0.0
// beat_counter : นับสะสม 1, 2, 3, ... ตลอด session
static float beat_pulse   = 0.0f;
static int   beat_counter = 0;

typedef struct { float bpm; float spo2; } line_task_arg_t;

// ─────────────────────────────────────────────────────────────────
//  Wi-Fi
// ─────────────────────────────────────────────────────────────────
static void wifi_event_handler(void *a, esp_event_base_t base, int32_t id, void *d)
{
    if      (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)        esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) { wifi_connected=false; esp_wifi_connect(); }
    else if (base == IP_EVENT   && id == IP_EVENT_STA_GOT_IP)         wifi_connected=true;
}

static void connect_wifi(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,    &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    wifi_config_t wc = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ─────────────────────────────────────────────────────────────────
//  LINE
// ─────────────────────────────────────────────────────────────────
static void send_line_notify(float bpm, float spo2)
{
    char bs[40], ss[40], body[300];
    if (bpm > 0) snprintf(bs, sizeof(bs), "Heart Rate = %.0f BPM", bpm);
    else         snprintf(bs, sizeof(bs), "Heart Rate = --");
    if (spo2 >= SPO2_MIN_VALID && spo2 <= SPO2_MAX_VALID)
                 snprintf(ss, sizeof(ss), "SpO2 = %.1f %%", spo2);
    else         snprintf(ss, sizeof(ss), "SpO2 = --");
    snprintf(body, sizeof(body),
             "{\"messages\":[{\"type\":\"text\",\"text\":\"%s\\n%s\"}]}", bs, ss);

    esp_http_client_config_t cfg = {
        .url            = "https://api.line.me/v2/bot/message/broadcast",
        .method         = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms     = 5000,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    char auth[300]; snprintf(auth, sizeof(auth), "Bearer %s", LINE_TOKEN);
    esp_http_client_set_header(c, "Content-Type",  "application/json");
    esp_http_client_set_header(c, "Authorization", auth);
    esp_http_client_set_post_field(c, body, strlen(body));
    esp_err_t err = esp_http_client_perform(c);
    if (err == ESP_OK) ESP_LOGI(TAG, "LINE status=%d", esp_http_client_get_status_code(c));
    else               ESP_LOGE(TAG, "LINE err=%s",    esp_err_to_name(err));
    esp_http_client_cleanup(c);
}

static void line_task(void *arg)
{
    line_task_arg_t *a = arg;
    send_line_notify(a->bpm, a->spo2);
    free(a); vTaskDelete(NULL);
}

// ─────────────────────────────────────────────────────────────────
//  MAX30102
// ─────────────────────────────────────────────────────────────────
static void max_write(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR<<1)|I2C_MASTER_WRITE, true);
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
    i2c_master_write_byte(cmd, (MAX30102_ADDR<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, REG_FIFO_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR<<1)|I2C_MASTER_READ, true);
    i2c_master_read(cmd, d, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    *red = (((uint32_t)d[0]<<16)|((uint32_t)d[1]<<8)|d[2]) & 0x3FFFF;
    *ir  = (((uint32_t)d[3]<<16)|((uint32_t)d[4]<<8)|d[5]) & 0x3FFFF;
}

static uint32_t max_read_ir_only(void)
{ uint32_t r,ir; max_read_sample(&r,&ir); return ir; }

static void max30102_init(void)
{
    max_write(REG_MODE_CONFIG, 0x40); vTaskDelay(100/portTICK_PERIOD_MS);
    max_write(REG_FIFO_CONFIG, 0x4F);
    max_write(REG_MODE_CONFIG, 0x03);
    max_write(REG_SPO2_CONFIG, 0x27);
    max_write(REG_LED1_PA, LED_PA);
    max_write(REG_LED2_PA, LED_PA);
}

// ─────────────────────────────────────────────────────────────────
//  Reset
// ─────────────────────────────────────────────────────────────────
static void reset_filters(uint32_t ir_seed, uint32_t red_seed)
{
    dc_ir=ir_seed; 
    dc_red=red_seed; 
    lp_ir=lp_red=0;
    sample_count=0; 
    sat_warned=false; 
    motion_count=motion_total=0;
    spo2_sum_ac_red=spo2_sum_dc_red=spo2_sum_ac_ir=spo2_sum_dc_ir=0;
    spo2_n=spo2_r_slot_idx=0;
    for (int i=0;i<SPO2_R_SLOTS;i++) spo2_r_slots[i]=-1.0f;
}

static void reset_beats(void)
{
    beat_count=peak_hist_cnt=0; last_beat_ms=last_peak_ms=0;
    peak_val=peak_sum=0; is_rising=false; auto_threshold=THRESHOLD_MIN;
    bpm_window_idx=bpm_window_cnt=0;
    beat_pulse=0.0f; beat_counter=0;   // ← reset ตัวนับ beat
    for (int i=0;i<40;i++)         beat_bpm[i]=0;
    for (int i=0;i<BPM_WINDOW;i++) bpm_window_buf[i]=0;
}

// ─────────────────────────────────────────────────────────────────
//  Signal processing
// ─────────────────────────────────────────────────────────────────
static bool is_finger_on(uint32_t ir)
{
    static bool on=false;
    if (!on && ir>=FINGER_ON_THRESHOLD)  on=true;
    if ( on && ir< FINGER_OFF_THRESHOLD) on=false;
    return on;
}

static bool check_motion(float sig, uint32_t ir)
{
    if (fabsf(sig) > MOTION_THRESHOLD) {
        if (++motion_count >= MOTION_CONSEC) {
            motion_total++; motion_count=0;
            dc_ir=ir; lp_ir=0; auto_threshold=THRESHOLD_MIN; is_rising=false; peak_val=0;
            ESP_LOGW(TAG, "Motion #%d", motion_total);
            return true;
        }
    } else { motion_count=0; }
    return false;
}

static bool detect_peak(float sig, uint32_t now_ms)
{
    auto_threshold *= THRESHOLD_DECAY;
    if (auto_threshold < THRESHOLD_MIN) auto_threshold=THRESHOLD_MIN;

    if (sig > peak_val) { peak_val=sig; is_rising=true; return false; }
    if (!is_rising || (peak_val-sig) <= auto_threshold) return false;

    uint32_t since = last_peak_ms ? (now_ms-last_peak_ms) : 9999;
    bool ok = (since >= REFRACTORY_MS)
           && (peak_val >= THRESHOLD_MIN*AMP_MIN_MULT)
           && (peak_hist_cnt < 3 || (peak_val/(peak_sum/peak_hist_cnt)) <= PEAK_RATIO_MAX);

    if (ok) {
        peak_sum+=peak_val; peak_hist_cnt++;
        float nt=peak_val*THRESHOLD_RATIO;
        auto_threshold = nt>THRESHOLD_MAX?THRESHOLD_MAX:(nt>THRESHOLD_MIN?nt:THRESHOLD_MIN);
        last_peak_ms=now_ms;

        // ── นับ beat: counter +1, pulse ตั้งเป็น 1.0 ──────────────
        beat_counter++;
        beat_pulse = 1.0f;

        ESP_LOGI(TAG, "BEAT peak=%.1f thr=%.1f RR=%lums beat#%d",
                 peak_val, auto_threshold, (unsigned long)since, beat_counter);
    }
    is_rising=false; peak_val=sig;
    return ok;
}

static float calc_live_bpm(void)
{
    if (!bpm_window_cnt) return 0;
    float s=0; for (int i=0;i<bpm_window_cnt;i++) s+=bpm_window_buf[i];
    return s/bpm_window_cnt;
}

static float calc_result_bpm(void)
{
    if (beat_count < 2) return 0;
    float s[40]; int n=0;
    for (int i=0;i<beat_count&&i<40;i++)
        if (beat_bpm[i]>30&&beat_bpm[i]<200) s[n++]=beat_bpm[i];
    if (n < 2) return 0;
    for (int i=0;i<n-1;i++)
        for (int j=0;j<n-1-i;j++)
            if (s[j]>s[j+1]) { float t=s[j]; s[j]=s[j+1]; s[j+1]=t; }
    float med=s[n/2], sum=0; int cnt=0;
    for (int i=0;i<n;i++)
        if (fabsf(s[i]-med)/med <= 0.20f) { sum+=s[i]; cnt++; }
    return cnt?sum/cnt:med;
}

static float calc_spo2(void)
{
    if (spo2_n < 100) return -1.0f;

    float vr[SPO2_R_SLOTS]; int nv=0;
    for (int i=0;i<spo2_r_slot_idx;i++)
        if (spo2_r_slots[i]>=SPO2_R_MIN && spo2_r_slots[i]<=SPO2_R_MAX)
            vr[nv++]=spo2_r_slots[i];

    float R;
    if (nv >= 3) {
        for (int i=0;i<nv-1;i++)
            for (int j=0;j<nv-1-i;j++)
                if (vr[j]>vr[j+1]) { float t=vr[j]; vr[j]=vr[j+1]; vr[j+1]=t; }
        float med=vr[nv/2], sum=0; int cnt=0;
        for (int i=0;i<nv;i++)
            if (fabsf(vr[i]-med)/med <= 0.20f) { sum+=vr[i]; cnt++; }
        R = cnt?sum/cnt:med;
    } else {
        double ra=sqrt(spo2_sum_ac_red/spo2_n), ia=sqrt(spo2_sum_ac_ir/spo2_n);
        double rd=spo2_sum_dc_red/spo2_n,       id=spo2_sum_dc_ir/spo2_n;
        if (rd<100||id<100||ia<0.1) return -1.0f;
        R=(float)((ra/rd)/(ia/id));
        if (R<SPO2_R_MIN||R>SPO2_R_MAX) return -1.0f;
    }

    float spo2=SPO2_A-SPO2_B*R;
    ESP_LOGI(TAG, "SpO2: R=%.4f -> %.1f%% (slots=%d)", R, spo2, nv);
    return (spo2>=SPO2_MIN_VALID && spo2<=SPO2_MAX_VALID) ? spo2 : -1.0f;
}

// ─────────────────────────────────────────────────────────────────
//  OLED (ไม่มีการเปลี่ยนแปลง)
// ─────────────────────────────────────────────────────────────────
static void screen_idle(void)
{
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, " Heart Rate Mon ", 16, true);
    ssd1306_display_text(&dev, 2, "  Place finger  ", 16, false);
    ssd1306_display_text(&dev, 3, "   on sensor    ", 16, false);
    ssd1306_display_text(&dev, 5, "  HR + SpO2     ", 16, false);
    ssd1306_display_text(&dev, 6, "  measurement   ", 16, false);
}

static void screen_settling(int sec_left, uint32_t ir)
{
    char buf[17];
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, "  Preparing...  ", 16, true);
    ssd1306_display_text(&dev, 2, "  Keep finger   ", 16, false);
    ssd1306_display_text(&dev, 3, "  still please  ", 16, false);
    unsigned long irk = ir/1000; if (irk>99999) irk=99999;
    snprintf(buf, sizeof(buf), "  IR:%5luk     ", irk); buf[16]='\0';
    ssd1306_display_text(&dev, 4, buf, 16, false);
    if (sec_left > 0) { memcpy(buf,"   Wait  0s...  ",17); buf[9]='0'+(char)(sec_left%10); }
    else              { memcpy(buf,"  Starting...   ",17); }
    ssd1306_display_text(&dev, 6, buf, 16, false);
}

static void screen_measuring(int sample, int beats, float live_bpm, int motions)
{
    char tmp[17];
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, "  Measuring...  ", 16, true);

    int pct=(sample*100)/MEASURE_SAMPLES, bars=(pct*12)/100;
    char bar[17]; bar[0]='[';
    for (int i=0;i<12;i++) bar[1+i]=(i<bars)?'#':'-';
    bar[13]=']'; bar[14]=' '; bar[15]=' '; bar[16]='\0';
    ssd1306_display_text(&dev, 1, bar, 16, false);

    memcpy(tmp,"    000%        ",17);
    int p=(pct>100)?100:(pct<0?0:pct);
    tmp[4]='0'+(char)(p/100); tmp[5]='0'+(char)((p/10)%10); tmp[6]='0'+(char)(p%10);
    ssd1306_display_text(&dev, 2, tmp, 16, false);

    snprintf(tmp,sizeof(tmp),"  Beats:  %3d   ",beats);
    ssd1306_display_text(&dev, 3, tmp, 16, false);

    if (live_bpm>0) snprintf(tmp,sizeof(tmp),"  Live: %3.0f BPM ",live_bpm);
    else            memcpy(tmp,"  Counting...   ",17);
    ssd1306_display_text(&dev, 4, tmp, 16, false);

    ssd1306_display_text(&dev, 5, "  SpO2: calc... ", 16, false);

    if (motions>0) snprintf(tmp,sizeof(tmp)," Motion:  %02d evt",motions>99?99:motions);
    else           memcpy(tmp,"                ",17);
    ssd1306_display_text(&dev, 6, tmp, 16, false);
}

static void screen_result(float bpm, float spo2, int motions)
{
    char tmp[17];
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, "    Result      ", 16, true);

    if (bpm>0) snprintf(tmp,sizeof(tmp),"  HR:  %3.0f BPM  ",bpm);
    else       memcpy(tmp,"  HR:   -- BPM  ",17);
    tmp[16]='\0'; ssd1306_display_text(&dev, 2, tmp, 16, false);

    if (spo2>=SPO2_MIN_VALID&&spo2<=SPO2_MAX_VALID) snprintf(tmp,sizeof(tmp),"  SpO2: %4.1f %%  ",spo2);
    else                                             memcpy(tmp,"  SpO2:  -- %   ",17);
    tmp[16]='\0'; ssd1306_display_text(&dev, 3, tmp, 16, false);

    if      (spo2>=95.0f)          ssd1306_display_text(&dev,4,"  Status: Normal",16,false);
    else if (spo2>=90.0f)          ssd1306_display_text(&dev,4,"  Status: Low   ",16,false);
    else if (spo2>=SPO2_MIN_VALID) ssd1306_display_text(&dev,4,"  Status: Danger",16,false);

    if (motions>0) snprintf(tmp,sizeof(tmp)," Motion:  %02d evt",motions>99?99:motions);
    else           memcpy(tmp,"  Sent to LINE  ",17);
    ssd1306_display_text(&dev, 6, tmp, 16, false);
}

static void screen_failed(void)
{
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, "  Meas. Failed  ", 16, true);
    ssd1306_display_text(&dev, 2, " No pulse found ", 16, false);
    ssd1306_display_text(&dev, 4, "  Keep still &  ", 16, false);
    ssd1306_display_text(&dev, 5, "  press gently, ", 16, false);
    ssd1306_display_text(&dev, 6, "  then retry    ", 16, false);
}

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
    screen_idle();

    // ── Serial Plotter header (7 channels) ───────────────────────
    // IR_AC     : waveform สัญญาณ IR — ดูรูปคลื่น pulse
    // RED_AC    : waveform สัญญาณ RED — ดู SpO2 component
    // LiveBPM   : BPM live (หาร 300 ให้ scale ใกล้เคียง waveform)
    // Threshold : ระดับ threshold ของ beat detector
    // R_ratio   : SpO2 R-ratio (ปกติ 0.4–0.8)
    // BeatPulse : spike 1.0 ทุกครั้งที่หัวใจเต้น 1 ครั้ง → เห็นเป็นแท่งพุ่ง
    // BeatCount : ตัวนับสะสม 1,2,3,... (หาร 100 ให้ scale พอดีกราฟ)
    printf(">IR_AC RED_AC LiveBPM Threshold R_ratio BeatPulse BeatCount\n");

    while (1) {
        uint32_t raw_red, raw_ir;
        max_read_sample(&raw_red, &raw_ir);
        uint32_t now_ms = (uint32_t)(esp_timer_get_time()/1000ULL);
        bool     finger = is_finger_on(raw_ir);

        if (state == STATE_IDLE) {
            if (finger) { finger_first_seen_ms=now_ms; state=STATE_WAIT_FINGER; }
        }

        else if (state == STATE_WAIT_FINGER) {
            if (!finger) { state=STATE_IDLE; screen_idle(); }
            else if (now_ms-finger_first_seen_ms >= FINGER_DEBOUNCE_MS) {
                if (raw_ir > SAT_THRESHOLD) { screen_settling(0,raw_ir); vTaskDelay(2000/portTICK_PERIOD_MS); }
                reset_filters(raw_ir, raw_red);
                reset_beats();
                state=STATE_SETTLING;
                screen_settling(5, raw_ir);
            }
        }

        else if (state == STATE_SETTLING) {
            if (!finger) { state=STATE_IDLE; screen_idle(); vTaskDelay(10/portTICK_PERIOD_MS); continue; }

            dc_ir  += ((float)raw_ir  - dc_ir)  * DC_ALPHA_FAST;
            dc_red += ((float)raw_red - dc_red)  * DC_ALPHA_FAST;
            lp_ir   = lp_ir  * (1.0f-LP_ALPHA) + ((float)raw_ir  - dc_ir)  * LP_ALPHA;
            lp_red  = lp_red * (1.0f-LP_ALPHA) + ((float)raw_red - dc_red) * LP_ALPHA;
            sample_count++;

            if (sample_count % 50 == 0)
                screen_settling((SETTLE_SAMPLES-sample_count)/100, raw_ir);

            printf("%f %f %f %f %f %f %f\n",
                   lp_ir/10000.0f,
                   lp_red/10000.0f,
                   0.0f,
                   auto_threshold/10000.0f,
                   0.0f,
                   0.0f,    // BeatPulse = 0 ช่วง settling
                   0.0f);   // BeatCount = 0 ช่วง settling

            if (sample_count >= SETTLE_SAMPLES) {
                reset_beats();
                sample_count=0; peak_val=fabsf(lp_ir); auto_threshold=THRESHOLD_MIN;
                state=STATE_MEASURING;
            }
        }

        else if (state == STATE_MEASURING) {
            if (!finger) { state=STATE_IDLE; screen_idle(); vTaskDelay(10/portTICK_PERIOD_MS); continue; }

            float dca = (sample_count<TRANSITION_SAMPLES) ? DC_ALPHA_MID : DC_ALPHA_SLOW;
            dc_ir  += ((float)raw_ir  - dc_ir)  * dca;
            dc_red += ((float)raw_red - dc_red)  * dca;
            float ac_ir  = (float)raw_ir  - dc_ir;
            float ac_red = (float)raw_red - dc_red;
            lp_ir   = lp_ir  * (1.0f-LP_ALPHA) + ac_ir  * LP_ALPHA;
            lp_red  = lp_red * (1.0f-LP_ALPHA) + ac_red * LP_ALPHA;

            if (sample_count >= TRANSITION_SAMPLES) {
                spo2_sum_ac_red += (double)lp_red*lp_red;
                spo2_sum_dc_red += (double)dc_red;
                spo2_sum_ac_ir  += (double)lp_ir *lp_ir;
                spo2_sum_dc_ir  += (double)dc_ir;
                spo2_n++;
                if (spo2_n%100==0 && spo2_r_slot_idx<SPO2_R_SLOTS) {
                    double ra=sqrt(spo2_sum_ac_red/spo2_n), ia=sqrt(spo2_sum_ac_ir/spo2_n);
                    double rd=spo2_sum_dc_red/spo2_n,       id=spo2_sum_dc_ir/spo2_n;
                    if (rd>100&&id>100&&ia>0.1)
                        spo2_r_slots[spo2_r_slot_idx++]=(float)((ra/rd)/(ia/id));
                }
            }

            // ── Detect beat ───────────────────────────────────────
            if (!check_motion(lp_ir,raw_ir) && detect_peak(lp_ir,now_ms)) {
                // beat_counter และ beat_pulse ถูก set ใน detect_peak แล้ว
                uint32_t rr=now_ms-last_beat_ms;
                if (last_beat_ms && rr>=MIN_RR_MS && rr<=MAX_RR_MS) {
                    float ibpm=60000.0f/rr;
                    if (beat_count<40) beat_bpm[beat_count++]=ibpm;
                    bpm_window_buf[bpm_window_idx]=ibpm;
                    bpm_window_idx=(bpm_window_idx+1)%BPM_WINDOW;
                    if (bpm_window_cnt<BPM_WINDOW) bpm_window_cnt++;
                }
                last_beat_ms=now_ms;
            }

            sample_count++;
            if (sample_count%100==0)
                screen_measuring(sample_count, beat_count, calc_live_bpm(), motion_total);

            // ── R-ratio live ──────────────────────────────────────
            float r_live = 0.0f;
            if (spo2_n >= 50) {
                double ra=sqrt(spo2_sum_ac_red/spo2_n), ia=sqrt(spo2_sum_ac_ir/spo2_n);
                double rd=spo2_sum_dc_red/spo2_n,       id=spo2_sum_dc_ir/spo2_n;
                if (rd>100&&id>100&&ia>0.1) r_live=(float)((ra/rd)/(ia/id));
            }

            // ── Serial Plotter output (7 channels) ───────────────
            printf("%f %f %f %f %f %f %f\n",
                   lp_ir/10000.0f,              // IR_AC
                   lp_red/10000.0f,             // RED_AC
                   calc_live_bpm()/300.0f,      // LiveBPM (scaled)
                   auto_threshold/10000.0f,     // Threshold
                   r_live,                      // R_ratio
                   beat_pulse,                  // BeatPulse: 1.0 ทุก beat แล้วกลับ 0
                   (float)beat_counter/100.0f); // BeatCount: 0.01, 0.02, 0.03, ...

            // ── reset beat_pulse หลัง print 1 sample ──────────────
            beat_pulse = 0.0f;

            if (sample_count>=MEASURE_SAMPLES) state=STATE_RESULT;
        }

        else if (state == STATE_RESULT) {
            float bpm  = calc_result_bpm();
            float spo2 = calc_spo2();
            bool  bpm_ok  = (beat_count>=MIN_BEATS && bpm>0);
            bool  spo2_ok = (spo2>=SPO2_MIN_VALID);

            double R_final = -1.0;
            if (spo2_n > 0) {
                double ra=sqrt(spo2_sum_ac_red/spo2_n), ia=sqrt(spo2_sum_ac_ir/spo2_n);
                double rd=spo2_sum_dc_red/spo2_n,       id=spo2_sum_dc_ir/spo2_n;
                if (rd>0&&id>0&&ia>0) R_final=(ra/rd)/(ia/id);
            }

            ESP_LOGI(TAG, "==============================");
            ESP_LOGI(TAG, "  RESULT BPM   : %.1f",   bpm);
            ESP_LOGI(TAG, "  RESULT SpO2  : %.1f%%",  spo2);
            ESP_LOGI(TAG, "  R-ratio      : %.4f  (normal ~0.5-0.7)", R_final);
            ESP_LOGI(TAG, "  BEATS (valid): %d", beat_count);
            ESP_LOGI(TAG, "  BEATS (total): %d", beat_counter);
            ESP_LOGI(TAG, "  MOTION       : %d events",   motion_total);
            ESP_LOGI(TAG, "  SPO2_N       : %d samples",  spo2_n);
            ESP_LOGI(TAG, "  SPO2_A/B     : %.0f / %.0f", (double)SPO2_A, (double)SPO2_B);
            ESP_LOGI(TAG, "==============================");

            if (bpm_ok || spo2_ok) {
                screen_result(bpm_ok?bpm:-1.0f, spo2, motion_total);
                line_task_arg_t *arg = malloc(sizeof(line_task_arg_t));
                if (arg) {
                    arg->bpm=bpm_ok?bpm:-1.0f; arg->spo2=spo2_ok?spo2:-1.0f;
                    xTaskCreate(line_task,"line",8192,arg,5,NULL);
                }
            } else { screen_failed(); }

            printf("%f %f %f %f %f %f %f\n",
                   0.0f, 0.0f,
                   bpm/300.0f,
                   0.0f,
                   (float)R_final,
                   0.0f,
                   (float)beat_counter/100.0f);

            while (is_finger_on(max_read_ir_only())) vTaskDelay(100/portTICK_PERIOD_MS);
            vTaskDelay(300/portTICK_PERIOD_MS);
            state=STATE_IDLE; screen_idle();
        }

        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}