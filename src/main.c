#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SENSOR_JEMBATAN";

#if CONFIG_IDF_TARGET_ESP32S3
  #define I2C_SDA_IO        8
  #define I2C_SCL_IO        9
#else
  #define I2C_SDA_IO        22   
  #define I2C_SCL_IO        21
#endif
#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         400000
#define I2C_TIMEOUT         pdMS_TO_TICKS(50)

#define MPU_ADDR            0x68   
#define REG_SMPLRT_DIV      0x19
#define REG_ACCEL_CONFIG    0x1C   
#define REG_ACCEL_CONFIG2   0x1D    
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75
#define ACCEL_LSB_PER_G     16384.0f   

#define FS_HZ               100        
#define SAMPLES             512        
#define PRE_SAMPLES         64       
#define ACCEL_AXIS          2          
#define ACCEL_DLPF_CFG      3          
#define TRIGGER_COUNTS      1600     
#define HOLDOFF_SAMPLES     300        
#define WARMUP_SAMPLES      200       
#define BAND_MIN_HZ         0.5f
#define BAND_MAX_HZ         40.0f
#define TAPER_FRAC          0.2f       
#define N_PEAKS             3
#define FFT_SELFTEST        0        

#if CONFIG_FREERTOS_HZ < 100
  #error "CONFIG_FREERTOS_HZ minimal 100 untuk sampling 100 Hz"
#endif

#define TRY(x) do { esp_err_t _e = (x); if (_e != ESP_OK) return _e; } while (0)

static int16_t ring[SAMPLES];
static float   vReal[SAMPLES], vImag[SAMPLES];
static float   tw_cos[SAMPLES / 2], tw_sin[SAMPLES / 2];

static esp_err_t i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    TRY(i2c_param_config(I2C_PORT, &conf));
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

static esp_err_t mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_write_to_device(I2C_PORT, MPU_ADDR, b, 2, I2C_TIMEOUT);
}

static esp_err_t mpu_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, MPU_ADDR, &reg, 1, buf, len, I2C_TIMEOUT);
}

static esp_err_t mpu_init(void)
{
    uint8_t id = 0;
    TRY(mpu_read(REG_WHO_AM_I, &id, 1));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (MPU6500 asli biasanya 0x70)", id);
    if (id != 0x70) {
        ESP_LOGW(TAG, "ID tidak 0x70: cek modul/klon dan wiring, lanjut dengan risiko sendiri");
    }
    TRY(mpu_write(REG_PWR_MGMT_1, 0x80));          
    vTaskDelay(pdMS_TO_TICKS(100));
    TRY(mpu_write(REG_PWR_MGMT_1, 0x01));          
    vTaskDelay(pdMS_TO_TICKS(50));
    TRY(mpu_write(REG_SMPLRT_DIV, 0x00));         
    TRY(mpu_write(REG_ACCEL_CONFIG, 0x00));       
    TRY(mpu_write(REG_ACCEL_CONFIG2, ACCEL_DLPF_CFG & 0x07));
    return ESP_OK;
}

static bool read_accel(int16_t *out)
{
    uint8_t d[6];
    if (mpu_read(REG_ACCEL_XOUT_H, d, 6) != ESP_OK) return false;
    int off = ACCEL_AXIS * 2;
    *out = (int16_t)(((uint16_t)d[off] << 8) | d[off + 1]);
    return true;
}


static void fft_init(void)
{
    for (int k = 0; k < SAMPLES / 2; k++) {
        float a = 2.0f * (float)M_PI * (float)k / (float)SAMPLES;
        tw_cos[k] = cosf(a);
        tw_sin[k] = sinf(a);
    }
}

static void fft(float *re, float *im, int n)
{
    int j = 0;
    for (int i = 0; i < n - 1; i++) {             
        if (i < j) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
        int k = n >> 1;
        while (k <= j) { j -= k; k >>= 1; }
        j += k;
    }
    for (int half = 1; half < n; half <<= 1) {   
        int step = n / (half * 2);
        for (int base = 0; base < n; base += half * 2) {
            for (int k = 0; k < half; k++) {
                float c = tw_cos[k * step];
                float s = -tw_sin[k * step];    
                int p = base + k, q = p + half;
                float tr = c * re[q] - s * im[q];
                float ti = s * re[q] + c * im[q];
                re[q] = re[p] - tr;  im[q] = im[p] - ti;
                re[p] += tr;         im[p] += ti;
            }
        }
    }
}


typedef struct { float f; float m; } peak_t;

static void process_event(int head, float fs_meas)
{
    double sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (float)ring[(head + i) % SAMPLES] / ACCEL_LSB_PER_G;
        sum += vReal[i];
    }
    float mean = (float)(sum / SAMPLES);         

    int taper_start = (int)((float)SAMPLES * (1.0f - TAPER_FRAC));
    float peak_g = 0.0f;
    for (int i = 0; i < SAMPLES; i++) {
        float x = vReal[i] - mean;
        if (fabsf(x) > peak_g) peak_g = fabsf(x);
        float w = 1.0f;
        if (i >= taper_start) {
            w = 0.5f * (1.0f + cosf((float)M_PI * (float)(i - taper_start) / (float)(SAMPLES - taper_start)));
        }
        vReal[i] = x * w;
        vImag[i] = 0.0f;
    }

    fft(vReal, vImag, SAMPLES);

    static float mag[SAMPLES / 2];
    for (int k = 0; k < SAMPLES / 2; k++) {
        mag[k] = sqrtf(vReal[k] * vReal[k] + vImag[k] * vImag[k]);
    }

    float df = fs_meas / (float)SAMPLES;
    int kmin = (int)ceilf(BAND_MIN_HZ / df);
    int kmax = (int)floorf(BAND_MAX_HZ / df);
    if (kmin < 1) kmin = 1;
    if (kmax > SAMPLES / 2 - 2) kmax = SAMPLES / 2 - 2;

    peak_t pk[N_PEAKS];
    memset(pk, 0, sizeof(pk));
    double band_sum = 0.0;
    int band_n = 0;
    for (int k = kmin; k <= kmax; k++) {
        band_sum += mag[k];
        band_n++;
        if (mag[k] > mag[k - 1] && mag[k] >= mag[k + 1]) {    
            float a = mag[k - 1], b = mag[k], c = mag[k + 1];
            float den = a - 2.0f * b + c;
            float delta = (fabsf(den) > 1e-12f) ? 0.5f * (a - c) / den : 0.0f; 
            peak_t p = { ((float)k + delta) * df, b };
            for (int i = 0; i < N_PEAKS; i++) {                
                if (p.m > pk[i].m) { peak_t t = pk[i]; pk[i] = p; p = t; }
            }
        }
    }
    float mean_mag = (band_n > 0) ? (float)(band_sum / band_n) : 1.0f;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "fs terukur = %.2f Hz | resolusi bin = %.3f Hz | puncak |a| = %.3f g", fs_meas, df, peak_g);
    for (int i = 0; i < N_PEAKS; i++) {
        if (pk[i].m > 0.0f) {
            ESP_LOGI(TAG, "Puncak %d: %.2f Hz  (rasio puncak/rata-rata pita = %.1f)", i + 1, pk[i].f, pk[i].m / mean_mag);
        }
    }
    ESP_LOGI(TAG, "========================================");
}

static void sensor_task(void *arg)
{
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(mpu_init());
    fft_init();

#if FFT_SELFTEST
    for (int i = 0; i < SAMPLES; i++) ring[i] = (int16_t)(4000.0f * sinf(2.0f * (float)M_PI * 10.0f * (float)i / (float)FS_HZ));
    ESP_LOGI(TAG, "SELFTEST: harus muncul puncak sekitar 10.00 Hz");
    process_event(0, (float)FS_HZ);
#endif

    const TickType_t period = pdMS_TO_TICKS(1000 / FS_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    int head = 0;                 
    float baseline = 0.0f;
    bool have_base = false;
    int warmup = WARMUP_SAMPLES, holdoff = 0;
    int post_left = -1;          
    int16_t last_good = 0;
    uint32_t n_total = 0, err_count = 0;
    int64_t t0 = esp_timer_get_time();

    ESP_LOGI(TAG, "Sistem siap. Menunggu pukulan aktuator...");

    for (;;) {
        vTaskDelayUntil(&last_wake, period);     

        int16_t x;
        if (read_accel(&x)) {
            last_good = x;
        } else {
            x = last_good;                         
            err_count++;
        }
        ring[head] = x;
        head = (head + 1) % SAMPLES;
        n_total++;
        if (!have_base) { baseline = (float)x; have_base = true; }

        if (post_left < 0) {                       
            float dev = (float)x - baseline;
            if (warmup == 0 && holdoff == 0 && fabsf(dev) > TRIGGER_COUNTS) {
                ESP_LOGW(TAG, "Trigger! Mengumpulkan %d sampel pasca-trigger...", SAMPLES - PRE_SAMPLES - 1);
                post_left = SAMPLES - PRE_SAMPLES - 1;
            } else {
                if (fabsf(dev) < TRIGGER_COUNTS) {
                    baseline += ((warmup > 0) ? 0.05f : 0.005f) * dev;  
                }
                if (warmup > 0) warmup--;
                if (holdoff > 0) holdoff--;
            }
        } else if (--post_left == 0) {            
            int64_t dt_us = esp_timer_get_time() - t0;
            float fs_meas = (dt_us > 0) ? (float)n_total * 1e6f / (float)dt_us : (float)FS_HZ;
            if (err_count) ESP_LOGW(TAG, "Ada %u kegagalan baca I2C pada periode ini", (unsigned)err_count);

            process_event(head, fs_meas);          

            post_left = -1;
            holdoff = HOLDOFF_SAMPLES;
            err_count = 0;
            n_total = 0;
            last_wake = xTaskGetTickCount();       
            t0 = esp_timer_get_time();
            ESP_LOGI(TAG, "Sistem siap. Menunggu pukulan aktuator...");
        }
    }
}

void app_main(void)
{
    xTaskCreate(sensor_task, "sensor_task", 6144, NULL, 5, NULL);
}