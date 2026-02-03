/*
 * ============================================================================
 * MULTI-METHOD LEAK DETECTION - ESP32 FIRMWARE
 * ESP32-C6 + PCM5102A (DAC) + Analog ADC
 * ============================================================================
 * 
 * CRITICAL FIX: Center frequency changed from 30kHz to 5kHz
 *               (30kHz violates Nyquist at 20kHz sample rate!)
 * 
 * Wiring:
 *   ESP32-C6          PCM5102A
 *   ---------         --------
 *   GPIO 5  ────────► DIN
 *   GPIO 6  ────────► BCK
 *   GPIO 7  ────────► LCK
 *   3.3V    ────────► VCC
 *   GND     ────────► GND, FMT, XMT (all to GND!)
 *   
 *   GPIO 4  ◄──────── Hydrophone/Mic (analog input)
 * 
 * Commands (921600 baud):
 *   HELLO           - Connection test, responds "ACK"
 *   ZC <seed> <u> <N> - Generate ZC chirp with parameters
 *   TX              - Transmit chirp and capture response
 *   STATUS          - Show current configuration
 *   TONE            - Output 1kHz test tone for 2 seconds
 *   ADC             - Read ADC for 1 second and report stats
 * 
 * Author: KAUST / lilxvg
 * Date: January 2025
 */

#include <Arduino.h>
#include "driver/i2s.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define PIN_I2S_DOUT    5     // ESP32 → PCM5102A DIN
#define PIN_I2S_BCK     6     // Bit clock
#define PIN_I2S_WS      7     // Word select (LRCK)
#define ADC_PIN         4     // Analog input from hydrophone

// ============================================================================
// CONFIGURATION
// ============================================================================
#define BAUD_RATE       921600
#define SAMPLE_RATE     20000   // 20 kHz sample rate

// ZC Chirp parameters - FIXED: Must satisfy Nyquist (f < FS/2 = 10kHz)
#define ZC_CENTER_FREQ  5000.0f // 5 kHz center frequency (was 30kHz - WRONG!)

// Buffer sizes
#define MAX_ZC_LEN      10000
#define RX_LEN          2000    // Receive buffer length

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
int seed_val = 0;
int root_u = 1;
int Nzc = 1000;
bool zc_ok = false;

int16_t* zc_buf = nullptr;
uint16_t rx_buf[RX_LEN];

// ============================================================================
// I2S INITIALIZATION (Legacy driver - works on ESP32-C6)
// ============================================================================
void i2s_init() {
    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false
    };

    i2s_pin_config_t pins = {
        .bck_io_num = PIN_I2S_BCK,
        .ws_io_num = PIN_I2S_WS,
        .data_out_num = PIN_I2S_DOUT,
        .data_in_num = -1
    };

    esp_err_t ret = i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
    if (ret != ESP_OK) {
        Serial.printf("ERROR: i2s_driver_install failed: %d\n", ret);
        return;
    }
    
    ret = i2s_set_pin(I2S_NUM_0, &pins);
    if (ret != ESP_OK) {
        Serial.printf("ERROR: i2s_set_pin failed: %d\n", ret);
        return;
    }
    
    Serial.println("I2S initialized OK");
}

// ============================================================================
// ZC CHIRP GENERATION - FIXED CENTER FREQUENCY
// ============================================================================
void build_zc() {
    // Free existing buffer
    if (zc_buf) {
        free(zc_buf);
        zc_buf = nullptr;
    }
    
    // Validate parameters
    if (Nzc <= 0 || Nzc > MAX_ZC_LEN) {
        Serial.printf("ERROR: Invalid Nzc=%d (max %d)\n", Nzc, MAX_ZC_LEN);
        zc_ok = false;
        return;
    }
    
    // Allocate buffer
    zc_buf = (int16_t*)malloc(Nzc * sizeof(int16_t));
    if (!zc_buf) {
        Serial.println("ERROR: malloc failed");
        zc_ok = false;
        return;
    }
    
    // Initialize random generator with seed (must match MATLAB!)
    srand(seed_val);
    float ph0 = ((float)rand() / RAND_MAX) * 2.0f * PI;
    
    // Generate ZC chirp
    // Formula: sin(2*pi*f_center*n/FS - pi*u*n*(n+1)/Nzc + ph0)
    // FIXED: Using 5kHz center frequency instead of 30kHz
    for (int n = 0; n < Nzc; n++) {
        float carrier = 2.0f * PI * ZC_CENTER_FREQ * (float)n / (float)SAMPLE_RATE;
        float chirp = PI * (float)root_u * (float)n * (float)(n + 1) / (float)Nzc;
        float phase = carrier - chirp + ph0;
        float v = sinf(phase);
        zc_buf[n] = (int16_t)(v * 30000.0f);  // Scale to ~90% of int16 range
    }
    
    zc_ok = true;
    Serial.printf("ZC built: seed=%d, u=%d, N=%d, f_center=%.0f Hz\n", 
                  seed_val, root_u, Nzc, ZC_CENTER_FREQ);
}

// ============================================================================
// ADC CAPTURE - Timed sampling
// ============================================================================
void capture_rx() {
    // Calculate delay for accurate sample rate
    uint32_t delay_us = (uint32_t)(1000000.0f / (float)SAMPLE_RATE);
    
    unsigned long next_sample = micros();
    
    for (int i = 0; i < RX_LEN; i++) {
        // Wait for next sample time
        while (micros() < next_sample) {
            // Busy wait for precise timing
        }
        
        rx_buf[i] = analogRead(ADC_PIN);
        next_sample += delay_us;
    }
}

// ============================================================================
// SEND RX DATA - Binary format (little-endian 16-bit)
// ============================================================================
void send_rx() {
    // Send as raw binary - faster and more reliable
    Serial.write((uint8_t*)rx_buf, RX_LEN * sizeof(uint16_t));
    Serial.flush();
}

// ============================================================================
// TEST TONE - 1kHz for 2 seconds
// ============================================================================
void play_test_tone() {
    Serial.println("Playing 1kHz test tone...");
    
    const int tone_freq = 1000;
    const int duration_ms = 2000;
    const int num_samples = (SAMPLE_RATE * duration_ms) / 1000;
    
    // Generate and send in chunks
    const int chunk_size = 256;
    int16_t chunk[chunk_size];
    
    int sample_idx = 0;
    while (sample_idx < num_samples) {
        int samples_to_send = min(chunk_size, num_samples - sample_idx);
        
        for (int i = 0; i < samples_to_send; i++) {
            float t = (float)(sample_idx + i) / (float)SAMPLE_RATE;
            chunk[i] = (int16_t)(sinf(2.0f * PI * tone_freq * t) * 30000.0f);
        }
        
        size_t bytes_written;
        i2s_write(I2S_NUM_0, chunk, samples_to_send * sizeof(int16_t), 
                  &bytes_written, portMAX_DELAY);
        
        sample_idx += samples_to_send;
    }
    
    Serial.println("Tone complete");
}

// ============================================================================
// ADC TEST - Read for 1 second and show stats
// ============================================================================
void test_adc() {
    Serial.println("Testing ADC for 1 second...");
    
    const int test_samples = 1000;
    uint32_t sum = 0;
    uint16_t min_val = 4095;
    uint16_t max_val = 0;
    
    for (int i = 0; i < test_samples; i++) {
        uint16_t val = analogRead(ADC_PIN);
        sum += val;
        if (val < min_val) min_val = val;
        if (val > max_val) max_val = val;
        delay(1);
    }
    
    float avg = (float)sum / (float)test_samples;
    Serial.printf("ADC: min=%d, max=%d, avg=%.1f, range=%d\n", 
                  min_val, max_val, avg, max_val - min_val);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) {
        delay(10);
    }
    
    delay(1000);
    
    Serial.println();
    Serial.println("============================================");
    Serial.println("   MULTI-METHOD LEAK DETECTION SYSTEM");
    Serial.println("   ESP32-C6 + PCM5102A + Analog ADC");
    Serial.println("============================================");
    Serial.printf("Sample Rate: %d Hz\n", SAMPLE_RATE);
    Serial.printf("ZC Center Freq: %.0f Hz\n", ZC_CENTER_FREQ);
    Serial.printf("RX Buffer: %d samples\n", RX_LEN);
    Serial.println();
    
    // Initialize ADC
    analogReadResolution(12);  // 12-bit ADC (0-4095)
    Serial.println("ADC initialized (12-bit)");
    
    // Initialize I2S
    i2s_init();
    
    Serial.println();
    Serial.println("Commands: HELLO, ZC <seed> <u> <N>, TX, STATUS, TONE, ADC");
    Serial.println();
    Serial.println("READY");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        // HELLO - Connection test
        if (cmd.startsWith("HELLO")) {
            Serial.println("ACK");
            Serial.flush();
        }
        
        // ZC <seed> <u> <Nzc> - Generate ZC chirp
        else if (cmd.startsWith("ZC")) {
            int parsed = sscanf(cmd.c_str(), "ZC %d %d %d", &seed_val, &root_u, &Nzc);
            if (parsed == 3) {
                build_zc();
                if (zc_ok) {
                    Serial.println("OK");
                } else {
                    Serial.println("ERROR: ZC build failed");
                }
            } else {
                Serial.println("ERROR: Usage: ZC <seed> <u> <Nzc>");
            }
            Serial.flush();
        }
        
        // TX - Transmit chirp and capture response
        else if (cmd.startsWith("TX")) {
            if (!zc_ok) {
                Serial.println("ERROR: ZC not ready. Send ZC command first.");
                Serial.flush();
            } else {
                // Transmit ZC chirp
                size_t bytes_written;
                esp_err_t ret = i2s_write(I2S_NUM_0, zc_buf, Nzc * sizeof(int16_t),
                                          &bytes_written, pdMS_TO_TICKS(1000));
                
                if (ret == ESP_OK) {
                    // Small delay before capture to avoid direct coupling
                    delayMicroseconds(100);
                    
                    // Capture ADC response
                    capture_rx();
                    
                    // Send binary data
                    send_rx();
                } else {
                    Serial.printf("ERROR: I2S write failed: %d\n", ret);
                    Serial.flush();
                }
            }
        }
        
        // STATUS - Show configuration
        else if (cmd.startsWith("STATUS")) {
            Serial.println("--- STATUS ---");
            Serial.printf("Sample Rate: %d Hz\n", SAMPLE_RATE);
            Serial.printf("ZC Center Freq: %.0f Hz\n", ZC_CENTER_FREQ);
            Serial.printf("ZC Ready: %s\n", zc_ok ? "YES" : "NO");
            if (zc_ok) {
                Serial.printf("ZC Params: seed=%d, u=%d, N=%d\n", seed_val, root_u, Nzc);
            }
            Serial.printf("RX Buffer: %d samples\n", RX_LEN);
            Serial.println("--------------");
            Serial.flush();
        }
        
        // TONE - Play test tone
        else if (cmd.startsWith("TONE")) {
            play_test_tone();
            Serial.flush();
        }
        
        // ADC - Test ADC
        else if (cmd.startsWith("ADC")) {
            test_adc();
            Serial.flush();
        }
        
        // Unknown command
        else if (cmd.length() > 0) {
            Serial.printf("ERROR: Unknown command '%s'\n", cmd.c_str());
            Serial.flush();
        }
    }
    
    delay(1);
}
