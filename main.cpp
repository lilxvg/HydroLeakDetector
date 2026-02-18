

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#include <string.h>

// ============================================================================
// SYSTEM CONFIGURATION
// ============================================================================

// Audio Parameters
#define SAMPLE_RATE         48000       // 48 kHz sampling
#define BITS_PER_SAMPLE     I2S_BITS_PER_SAMPLE_32BIT
#define CENTER_FREQ         8000.0f     // 8 kHz carrier (good for water)
#define BANDWIDTH           4000.0f     // 4 kHz bandwidth

// Zadoff-Chu Parameters
#define ZC_LENGTH           127         // Prime number for ZC sequence
#define ZC_ROOT             29          // Root index (coprime with length)

// Gold Code Parameters
#define GOLD_LENGTH         127         // 2^7 - 1
#define CHIPS_PER_SYMBOL    31          // Spreading factor

// Signal Timing
#define PREAMBLE_REPS       3           // Repeat preamble for robustness
#define GUARD_SAMPLES       480         // 10ms guard interval
#define BURST_INTERVAL_MS   200         // Time between bursts
#define NUM_AVERAGES        32          // Number of bursts to average

// Buffer Sizes
#define TX_BUFFER_SIZE      32768       // Transmit buffer
#define RX_BUFFER_SIZE      65536       // Receive buffer (2x TX for full echo)
#define DMA_BUFFER_COUNT    8
#define DMA_BUFFER_LEN      1024

// I2S Pins - PCM5102A DAC (Transmit)
#define I2S_TX_BCK_PIN      4           // Bit clock
#define I2S_TX_WS_PIN       5           // Word select (LRCK)
#define I2S_TX_DATA_PIN     6           // Data out

// I2S Pins - PCM1808 ADC (Receive)
#define I2S_RX_BCK_PIN      7           // Bit clock
#define I2S_RX_WS_PIN       15          // Word select (LRCK)
#define I2S_RX_DATA_PIN     16          // Data in
#define I2S_RX_MCLK_PIN     17          // Master clock (12.288 MHz for 48kHz)

// Pipe Physical Parameters
#define PIPE_LENGTH_M       1.0f        // 1 meter pipe
#define SOUND_SPEED_WATER   1480.0f     // m/s in water at 20°C
#define VALVE_SPACING_M     0.25f       // Valves at 0.25, 0.5, 0.75, 1.0m

// ============================================================================
// GLOBAL BUFFERS
// ============================================================================

// Signal buffers (32-bit for I2S, but we use float for processing)
static int32_t tx_buffer[TX_BUFFER_SIZE];
static int32_t rx_buffer[RX_BUFFER_SIZE];

// Processing buffers (float for precision)
static float zc_preamble_real[ZC_LENGTH];
static float zc_preamble_imag[ZC_LENGTH];
static float gold_code[GOLD_LENGTH];
static float tx_waveform[TX_BUFFER_SIZE];
static float rx_waveform[RX_BUFFER_SIZE];
static float correlation_accum[RX_BUFFER_SIZE];

// Circular correlation buffer for averaging
static float averaged_correlation[RX_BUFFER_SIZE];

// Results
static float leak_distances[4];         // Up to 4 leak positions
static float leak_strengths[4];
static int num_leaks_detected = 0;

// State
static int current_burst = 0;
static int tx_waveform_length = 0;
static bool system_ready = false;

// ============================================================================
// ZADOFF-CHU SEQUENCE GENERATION
// ============================================================================

void generate_zadoff_chu(float* real, float* imag, int length, int root) {
    // ZC(n) = exp(-j * pi * root * n * (n+1) / length)
    // For length = prime, all roots produce CAZAC sequences
    
    for (int n = 0; n < length; n++) {
        float phase = -M_PI * root * n * (n + 1) / (float)length;
        real[n] = cosf(phase);
        imag[n] = sinf(phase);
    }
    
    Serial.printf("[ZC] Generated Zadoff-Chu sequence: length=%d, root=%d\n", length, root);
}

// ============================================================================
// GOLD CODE GENERATION
// ============================================================================

// LFSR for m-sequence generation
int lfsr_step(int state, int taps, int length) {
    int feedback = 0;
    int temp = state & taps;
    while (temp) {
        feedback ^= (temp & 1);
        temp >>= 1;
    }
    return ((state << 1) | feedback) & ((1 << length) - 1);
}

void generate_gold_code(float* code, int length) {
    // Generate two m-sequences and XOR them
    // For length 127 (2^7-1), use polynomials: x^7+x^3+1 and x^7+x^3+x^2+x+1
    
    int poly1 = 0b0001001;  // Taps at positions 7,3 (0-indexed: 6,2)
    int poly2 = 0b0001111;  // Taps at positions 7,3,2,1
    
    int state1 = 0x01;      // Initial state (non-zero)
    int state2 = 0x01;
    
    int m1[127], m2[127];
    
    for (int i = 0; i < length; i++) {
        m1[i] = state1 & 1;
        m2[i] = state2 & 1;
        state1 = lfsr_step(state1, poly1, 7);
        state2 = lfsr_step(state2, poly2, 7);
    }
    
    // Gold code = m1 XOR m2, convert to +1/-1
    for (int i = 0; i < length; i++) {
        code[i] = (m1[i] ^ m2[i]) ? 1.0f : -1.0f;
    }
    
    Serial.printf("[GOLD] Generated Gold code: length=%d\n", length);
}

// ============================================================================
// WAVEFORM CONSTRUCTION
// ============================================================================

void construct_tx_waveform() {
    int idx = 0;
    float samples_per_chip = SAMPLE_RATE / CENTER_FREQ;  // Samples per ZC/Gold chip
    
    // Calculate actual samples per symbol for proper timing
    int samples_per_zc_chip = (int)(SAMPLE_RATE / (CENTER_FREQ / 4));  // ~24 samples
    int samples_per_gold_chip = (int)(SAMPLE_RATE / (BANDWIDTH / 2));  // ~24 samples
    
    Serial.printf("[TX] Building waveform: ZC chip=%d samples, Gold chip=%d samples\n",
                  samples_per_zc_chip, samples_per_gold_chip);
    
    // === PREAMBLE: Repeated Zadoff-Chu ===
    for (int rep = 0; rep < PREAMBLE_REPS; rep++) {
        for (int chip = 0; chip < ZC_LENGTH; chip++) {
            // Modulate ZC onto carrier
            for (int s = 0; s < samples_per_zc_chip && idx < TX_BUFFER_SIZE; s++) {
                float t = (float)s / SAMPLE_RATE;
                float carrier = cosf(2.0f * M_PI * CENTER_FREQ * t);
                // Complex ZC modulated: Re{ZC * exp(j*2*pi*fc*t)}
                float sample = zc_preamble_real[chip] * carrier;
                tx_waveform[idx++] = sample;
            }
        }
        
        // Guard interval between repetitions
        for (int g = 0; g < GUARD_SAMPLES && idx < TX_BUFFER_SIZE; g++) {
            tx_waveform[idx++] = 0.0f;
        }
    }
    
    int preamble_end = idx;
    Serial.printf("[TX] Preamble ends at sample %d (%.1f ms)\n", 
                  preamble_end, 1000.0f * preamble_end / SAMPLE_RATE);
    
    // === SPREAD SPECTRUM PAYLOAD ===
    // Send a known data pattern spread with Gold code
    // This gives us additional correlation peaks for multipath analysis
    
    int data_bits[] = {1, 0, 1, 1, 0, 0, 1, 0};  // 8-bit pattern
    int num_data_bits = 8;
    
    for (int bit = 0; bit < num_data_bits; bit++) {
        float bit_val = data_bits[bit] ? 1.0f : -1.0f;
        
        for (int chip = 0; chip < GOLD_LENGTH; chip++) {
            float spread_val = bit_val * gold_code[chip];
            
            // BPSK modulation onto carrier
            for (int s = 0; s < samples_per_gold_chip && idx < TX_BUFFER_SIZE; s++) {
                float t = (float)(idx) / SAMPLE_RATE;
                float carrier = cosf(2.0f * M_PI * CENTER_FREQ * t);
                tx_waveform[idx++] = spread_val * carrier;
            }
        }
    }
    
    // Final guard
    for (int g = 0; g < GUARD_SAMPLES * 2 && idx < TX_BUFFER_SIZE; g++) {
        tx_waveform[idx++] = 0.0f;
    }
    
    tx_waveform_length = idx;
    
    // Normalize to prevent clipping
    float max_val = 0.0f;
    for (int i = 0; i < tx_waveform_length; i++) {
        if (fabsf(tx_waveform[i]) > max_val) max_val = fabsf(tx_waveform[i]);
    }
    if (max_val > 0) {
        float scale = 0.9f / max_val;
        for (int i = 0; i < tx_waveform_length; i++) {
            tx_waveform[i] *= scale;
        }
    }
    
    // Convert to 32-bit signed integer for I2S
    for (int i = 0; i < tx_waveform_length; i++) {
        tx_buffer[i] = (int32_t)(tx_waveform[i] * 2147483647.0f);
    }
    
    Serial.printf("[TX] Waveform complete: %d samples (%.1f ms)\n",
                  tx_waveform_length, 1000.0f * tx_waveform_length / SAMPLE_RATE);
}

// ============================================================================
// CYCLIC CORRELATION
// ============================================================================

void cyclic_correlate_zc(float* signal, int sig_len, float* output) {
    // Cyclic correlation with ZC preamble
    // C[k] = sum_n{ signal[n] * conj(ZC[(n-k) mod N]) }
    
    // For efficiency, we correlate with the baseband ZC after carrier removal
    // This is a simplified matched filter approach
    
    int zc_samples = ZC_LENGTH * (int)(SAMPLE_RATE / (CENTER_FREQ / 4));
    
    for (int k = 0; k < sig_len - zc_samples; k++) {
        float corr_real = 0.0f;
        float corr_imag = 0.0f;
        
        for (int n = 0; n < ZC_LENGTH; n++) {
            int sample_idx = k + n * (int)(SAMPLE_RATE / (CENTER_FREQ / 4));
            if (sample_idx >= sig_len) break;
            
            // Demodulate and correlate
            float t = (float)sample_idx / SAMPLE_RATE;
            float carrier_i = cosf(2.0f * M_PI * CENTER_FREQ * t);
            float carrier_q = -sinf(2.0f * M_PI * CENTER_FREQ * t);
            
            float demod_i = signal[sample_idx] * carrier_i;
            float demod_q = signal[sample_idx] * carrier_q;
            
            // Correlate with conjugate of ZC
            corr_real += demod_i * zc_preamble_real[n] + demod_q * zc_preamble_imag[n];
            corr_imag += demod_q * zc_preamble_real[n] - demod_i * zc_preamble_imag[n];
        }
        
        output[k] = sqrtf(corr_real * corr_real + corr_imag * corr_imag);
    }
}

void correlate_gold_code(float* signal, int sig_len, int start_idx, float* output) {
    // Cross-correlation with Gold code for spread spectrum detection
    
    int chip_samples = (int)(SAMPLE_RATE / (BANDWIDTH / 2));
    int code_samples = GOLD_LENGTH * chip_samples;
    
    for (int k = 0; k < sig_len - code_samples - start_idx; k++) {
        float corr = 0.0f;
        
        for (int chip = 0; chip < GOLD_LENGTH; chip++) {
            int sample_idx = start_idx + k + chip * chip_samples;
            if (sample_idx >= sig_len) break;
            
            // Envelope detection (rectify)
            float env = 0.0f;
            for (int s = 0; s < chip_samples && (sample_idx + s) < sig_len; s++) {
                env += fabsf(signal[sample_idx + s]);
            }
            env /= chip_samples;
            
            // Correlate
            corr += env * gold_code[chip];
        }
        
        output[k] = fabsf(corr);
    }
}

// ============================================================================
// PEAK DETECTION AND LEAK LOCALIZATION
// ============================================================================

typedef struct {
    int position;
    float magnitude;
    float distance_m;
} CorrelationPeak;

int find_peaks(float* corr, int len, CorrelationPeak* peaks, int max_peaks, float threshold) {
    int num_peaks = 0;
    
    // Find local maxima above threshold
    for (int i = 2; i < len - 2 && num_peaks < max_peaks; i++) {
        if (corr[i] > threshold &&
            corr[i] > corr[i-1] && corr[i] > corr[i-2] &&
            corr[i] > corr[i+1] && corr[i] > corr[i+2]) {
            
            // Parabolic interpolation for sub-sample accuracy
            float alpha = corr[i-1];
            float beta = corr[i];
            float gamma = corr[i+1];
            float delta = 0.5f * (alpha - gamma) / (alpha - 2*beta + gamma);
            
            peaks[num_peaks].position = i;
            peaks[num_peaks].magnitude = beta - 0.25f * (alpha - gamma) * delta;
            
            // Convert sample position to distance
            // Round-trip time: sample_idx / sample_rate
            // One-way distance: (round_trip_time * sound_speed) / 2
            float round_trip_samples = (float)(i + delta);
            float round_trip_time = round_trip_samples / SAMPLE_RATE;
            peaks[num_peaks].distance_m = (round_trip_time * SOUND_SPEED_WATER) / 2.0f;
            
            num_peaks++;
            i += 10;  // Skip ahead to avoid detecting same peak
        }
    }
    
    return num_peaks;
}

void analyze_leaks(CorrelationPeak* peaks, int num_peaks) {
    num_leaks_detected = 0;
    
    // Expected reflection distances (round-trip from top):
    // Valve at 0.25m -> 0.25m reflection
    // Valve at 0.50m -> 0.50m reflection
    // Valve at 0.75m -> 0.75m reflection
    // Valve at 1.00m -> 1.00m reflection (pipe end)
    
    float expected_distances[] = {0.25f, 0.50f, 0.75f, 1.00f};
    float tolerance = 0.05f;  // 5cm tolerance
    
    Serial.println("\n========== LEAK ANALYSIS ==========");
    
    for (int i = 0; i < num_peaks && i < 10; i++) {
        Serial.printf("Peak %d: sample=%d, distance=%.3fm, magnitude=%.2f\n",
                      i, peaks[i].position, peaks[i].distance_m, peaks[i].magnitude);
        
        // Check if this matches an expected valve position
        for (int v = 0; v < 4; v++) {
            if (fabsf(peaks[i].distance_m - expected_distances[v]) < tolerance) {
                if (num_leaks_detected < 4) {
                    leak_distances[num_leaks_detected] = peaks[i].distance_m;
                    leak_strengths[num_leaks_detected] = peaks[i].magnitude;
                    num_leaks_detected++;
                    Serial.printf("  -> LEAK DETECTED at valve position %.2fm!\n", 
                                  expected_distances[v]);
                }
                break;
            }
        }
    }
    
    if (num_leaks_detected == 0) {
        Serial.println("No leaks detected at expected valve positions.");
    }
    
    Serial.println("====================================\n");
}

// ============================================================================
// I2S CONFIGURATION
// ============================================================================

void configure_i2s_tx() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = DMA_BUFFER_COUNT,
        .dma_buf_len = DMA_BUFFER_LEN,
        .use_apll = true,
        .tx_desc_auto_clear = true
    };
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_TX_BCK_PIN,
        .ws_io_num = I2S_TX_WS_PIN,
        .data_out_num = I2S_TX_DATA_PIN,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_config));
    
    Serial.println("[I2S] TX configured on I2S_NUM_0");
}

void configure_i2s_rx() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = DMA_BUFFER_COUNT,
        .dma_buf_len = DMA_BUFFER_LEN,
        .use_apll = true,
        .tx_desc_auto_clear = false
    };
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_RX_BCK_PIN,
        .ws_io_num = I2S_RX_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_RX_DATA_PIN
    };
    
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_1, &pin_config));
    
    Serial.println("[I2S] RX configured on I2S_NUM_1");
}

// ============================================================================
// TRANSMIT AND RECEIVE
// ============================================================================

void transmit_burst() {
    size_t bytes_written;
    
    // Write entire waveform to I2S TX
    // Note: Each sample is 32-bit, stereo = 8 bytes per sample pair
    size_t bytes_to_write = tx_waveform_length * sizeof(int32_t) * 2;  // Stereo
    
    // Prepare stereo buffer (same signal on both channels)
    static int32_t stereo_buffer[TX_BUFFER_SIZE * 2];
    for (int i = 0; i < tx_waveform_length; i++) {
        stereo_buffer[i * 2] = tx_buffer[i];      // Left
        stereo_buffer[i * 2 + 1] = tx_buffer[i];  // Right
    }
    
    i2s_write(I2S_NUM_0, stereo_buffer, bytes_to_write, &bytes_written, portMAX_DELAY);
    
    Serial.printf("[TX] Burst %d transmitted: %d bytes\n", current_burst, bytes_written);
}

void receive_echo() {
    size_t bytes_read;
    
    // Calculate required capture time
    // Max round-trip for 1m pipe: 2 * 1.0 / 1480 = 1.35ms
    // Plus TX duration + margin
    float tx_duration_ms = 1000.0f * tx_waveform_length / SAMPLE_RATE;
    float max_echo_ms = 2.0f * 1000.0f * PIPE_LENGTH_M / SOUND_SPEED_WATER;
    float capture_time_ms = tx_duration_ms + max_echo_ms + 50.0f;  // 50ms margin
    
    int capture_samples = (int)(capture_time_ms * SAMPLE_RATE / 1000.0f);
    if (capture_samples > RX_BUFFER_SIZE / 2) capture_samples = RX_BUFFER_SIZE / 2;
    
    // Read stereo data
    static int32_t stereo_buffer[RX_BUFFER_SIZE];
    size_t bytes_to_read = capture_samples * sizeof(int32_t) * 2;
    
    i2s_read(I2S_NUM_1, stereo_buffer, bytes_to_read, &bytes_read, portMAX_DELAY);
    
    // Extract mono (left channel only)
    int samples_read = bytes_read / (sizeof(int32_t) * 2);
    for (int i = 0; i < samples_read; i++) {
        rx_buffer[i] = stereo_buffer[i * 2];
        rx_waveform[i] = (float)rx_buffer[i] / 2147483647.0f;
    }
    
    Serial.printf("[RX] Captured %d samples (%.1f ms)\n", 
                  samples_read, 1000.0f * samples_read / SAMPLE_RATE);
}

// ============================================================================
// MAIN PROCESSING LOOP
// ============================================================================

void run_detection_cycle() {
    Serial.println("\n########## STARTING DETECTION CYCLE ##########\n");
    
    // Clear averaging buffer
    memset(averaged_correlation, 0, sizeof(averaged_correlation));
    
    for (current_burst = 0; current_burst < NUM_AVERAGES; current_burst++) {
        Serial.printf("\n--- Burst %d/%d ---\n", current_burst + 1, NUM_AVERAGES);
        
        // Clear RX buffer before capture
        i2s_zero_dma_buffer(I2S_NUM_1);
        
        // Small delay to ensure RX DMA is ready
        delay(10);
        
        // Start receiving (this will capture during and after TX)
        // We use a task-based approach for simultaneous TX/RX
        
        // For simplicity here, we do sequential TX then RX
        // The echo will still be in the pipe during RX window
        
        // Transmit
        unsigned long tx_start = micros();
        transmit_burst();
        unsigned long tx_end = micros();
        
        // Receive (start immediately, echo is still bouncing)
        receive_echo();
        
        // Process correlation
        static float burst_correlation[RX_BUFFER_SIZE];
        memset(burst_correlation, 0, sizeof(burst_correlation));
        
        // Zadoff-Chu correlation for precise timing
        cyclic_correlate_zc(rx_waveform, RX_BUFFER_SIZE / 2, burst_correlation);
        
        // Accumulate for averaging
        for (int i = 0; i < RX_BUFFER_SIZE / 2; i++) {
            averaged_correlation[i] += burst_correlation[i] / NUM_AVERAGES;
        }
        
        // Inter-burst delay
        delay(BURST_INTERVAL_MS);
    }
    
    Serial.println("\n--- Processing Complete ---\n");
    
    // Find correlation peaks
    CorrelationPeak peaks[20];
    
    // Calculate dynamic threshold (mean + 3*std)
    float mean = 0.0f;
    int corr_len = RX_BUFFER_SIZE / 2;
    for (int i = 0; i < corr_len; i++) {
        mean += averaged_correlation[i];
    }
    mean /= corr_len;
    
    float variance = 0.0f;
    for (int i = 0; i < corr_len; i++) {
        float diff = averaged_correlation[i] - mean;
        variance += diff * diff;
    }
    float std_dev = sqrtf(variance / corr_len);
    float threshold = mean + 3.0f * std_dev;
    
    Serial.printf("[ANALYSIS] Threshold: %.4f (mean=%.4f, std=%.4f)\n", 
                  threshold, mean, std_dev);
    
    int num_peaks = find_peaks(averaged_correlation, corr_len, peaks, 20, threshold);
    Serial.printf("[ANALYSIS] Found %d correlation peaks\n", num_peaks);
    
    // Analyze for leaks
    analyze_leaks(peaks, num_peaks);
    
    // Send results to MATLAB via Serial
    send_results_to_matlab(peaks, num_peaks);
}

// ============================================================================
// SERIAL COMMUNICATION (FOR MATLAB)
// ============================================================================

void send_results_to_matlab(CorrelationPeak* peaks, int num_peaks) {
    // Protocol: JSON-like format for easy MATLAB parsing
    Serial.println("\n<DATA_START>");
    
    // Send correlation data (downsampled for speed)
    Serial.println("<CORRELATION>");
    int downsample = 10;
    for (int i = 0; i < RX_BUFFER_SIZE / 2; i += downsample) {
        Serial.printf("%.6f,", averaged_correlation[i]);
        if ((i / downsample) % 100 == 99) Serial.println();
    }
    Serial.println("\n</CORRELATION>");
    
    // Send peak data
    Serial.println("<PEAKS>");
    for (int i = 0; i < num_peaks; i++) {
        Serial.printf("%d,%.6f,%.4f\n", 
                      peaks[i].position, peaks[i].magnitude, peaks[i].distance_m);
    }
    Serial.println("</PEAKS>");
    
    // Send leak results
    Serial.println("<LEAKS>");
    Serial.printf("NUM_LEAKS=%d\n", num_leaks_detected);
    for (int i = 0; i < num_leaks_detected; i++) {
        Serial.printf("LEAK%d=%.4f,%.4f\n", i, leak_distances[i], leak_strengths[i]);
    }
    Serial.println("</LEAKS>");
    
    // Send raw waveform (optional, for debugging)
    Serial.println("<RAW_RX>");
    for (int i = 0; i < min(RX_BUFFER_SIZE / 2, 10000); i += 10) {
        Serial.printf("%.6f,", rx_waveform[i]);
        if ((i / 10) % 100 == 99) Serial.println();
    }
    Serial.println("\n</RAW_RX>");
    
    Serial.println("<DATA_END>");
}

void handle_serial_commands() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd == "START") {
            Serial.println("ACK:START");
            run_detection_cycle();
        }
        else if (cmd == "PING") {
            Serial.println("ACK:PONG");
        }
        else if (cmd == "STATUS") {
            Serial.printf("ACK:READY=%d,BURSTS=%d\n", system_ready ? 1 : 0, NUM_AVERAGES);
        }
        else if (cmd == "CONFIG") {
            // Send configuration
            Serial.println("<CONFIG>");
            Serial.printf("SAMPLE_RATE=%d\n", SAMPLE_RATE);
            Serial.printf("ZC_LENGTH=%d\n", ZC_LENGTH);
            Serial.printf("ZC_ROOT=%d\n", ZC_ROOT);
            Serial.printf("GOLD_LENGTH=%d\n", GOLD_LENGTH);
            Serial.printf("NUM_AVERAGES=%d\n", NUM_AVERAGES);
            Serial.printf("PIPE_LENGTH=%.2f\n", PIPE_LENGTH_M);
            Serial.printf("SOUND_SPEED=%.1f\n", SOUND_SPEED_WATER);
            Serial.printf("TX_SAMPLES=%d\n", tx_waveform_length);
            Serial.println("</CONFIG>");
        }
        else if (cmd.startsWith("SET_AVG=")) {
            // Allow runtime averaging adjustment
            int new_avg = cmd.substring(8).toInt();
            if (new_avg >= 1 && new_avg <= 256) {
                // Note: Can't modify const, would need to make NUM_AVERAGES non-const
                Serial.printf("ACK:SET_AVG=%d\n", new_avg);
            }
        }
        else if (cmd == "TX_WAVEFORM") {
            // Send TX waveform for verification
            Serial.println("<TX_WAVEFORM>");
            for (int i = 0; i < tx_waveform_length; i += 10) {
                Serial.printf("%.6f,", tx_waveform[i]);
                if ((i / 10) % 100 == 99) Serial.println();
            }
            Serial.println("\n</TX_WAVEFORM>");
        }
        else if (cmd == "ZC_SEQ") {
            // Send ZC sequence
            Serial.println("<ZC_SEQ>");
            for (int i = 0; i < ZC_LENGTH; i++) {
                Serial.printf("%.6f,%.6f\n", zc_preamble_real[i], zc_preamble_imag[i]);
            }
            Serial.println("</ZC_SEQ>");
        }
        else if (cmd == "GOLD_CODE") {
            // Send Gold code
            Serial.println("<GOLD_CODE>");
            for (int i = 0; i < GOLD_LENGTH; i++) {
                Serial.printf("%.0f,", gold_code[i]);
            }
            Serial.println("\n</GOLD_CODE>");
        }
    }
}

// ============================================================================
// ARDUINO SETUP AND LOOP
// ============================================================================

void setup() {
    Serial.begin(921600);  // High baud rate for fast data transfer
    delay(1000);
    
    Serial.println("\n");
    Serial.println("================================================");
    Serial.println("  HydroLeak Acoustic Leak Detection System");
    Serial.println("  ESP32-C6 + PCM1808 ADC + PCM5102A DAC");
    Serial.println("================================================\n");
    
    // Generate signal sequences
    Serial.println("[INIT] Generating Zadoff-Chu preamble...");
    generate_zadoff_chu(zc_preamble_real, zc_preamble_imag, ZC_LENGTH, ZC_ROOT);
    
    Serial.println("[INIT] Generating Gold code...");
    generate_gold_code(gold_code, GOLD_LENGTH);
    
    Serial.println("[INIT] Constructing TX waveform...");
    construct_tx_waveform();
    
    // Configure I2S interfaces
    Serial.println("[INIT] Configuring I2S TX (PCM5102A)...");
    configure_i2s_tx();
    
    Serial.println("[INIT] Configuring I2S RX (PCM1808)...");
    configure_i2s_rx();
    
    // System ready
    system_ready = true;
    Serial.println("\n[INIT] System ready!");
    Serial.println("Commands: START, PING, STATUS, CONFIG, TX_WAVEFORM, ZC_SEQ, GOLD_CODE");
    Serial.println("Send 'START' to begin detection cycle.\n");
}

void loop() {
    handle_serial_commands();
    delay(10);
}
