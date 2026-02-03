% ============================================================
% MULTI-METHOD ACOUSTIC LEAK DETECTION (FUSION FRAMEWORK)
% ============================================================
% • Zadoff–Chu excitation (ESP32 synchronized)
% • Cross-correlation detection
% • Energy envelope detection
% • Phase correlation detection
% • Multi-measurement fusion + confidence estimation
% ============================================================

clear; clc; close all;

%% ============================================================
% GLOBAL CONFIGURATION
% ============================================================

FS = 20000;             % Sampling frequency (Hz)
Nzc = 1000;             % Zadoff–Chu sequence length
RX_LEN = 2000;          % Number of RX samples
NUM_MEASUREMENTS = 10;  % Total acquisitions (>= 8 required)

c_water = 1480;         % Speed of sound in water (m/s)
ROUND_TRIP = true;      % Echo-based (round-trip) measurement
MAX_DISTANCE = 1.0;     % Pipe length (m)

% Validation thresholds
MIN_SNR_DB = 8;
MIN_CORRELATION = 30;
MIN_VALID_MEASUREMENTS = 8;

%% ============================================================
% ZC SIGNAL PARAMETERS (MUST MATCH ESP32)
% ============================================================
% Nyquist-safe: center frequency < FS/2

ZC_CENTER_FREQ = 5000;  % 5 kHz carrier

%% ============================================================
% SERIAL PORT SETUP
% ============================================================

ports = serialportlist("available");
fprintf("Available ports:\n");
for i = 1:numel(ports)
    fprintf("  %d: %s\n", i, ports(i));
end

idx = input("Select port number: ");
if idx < 1 || idx > numel(ports)
    error("Invalid port selection");
end

fprintf("\nOpening port %s...\n", ports(idx));
s = serialport(ports(idx), 921600, "Timeout", 10);
configureTerminator(s, "LF");
pause(2);

flush(s);
writeline(s, "HELLO");
pause(0.5);

% Handshake verification
if s.NumBytesAvailable > 0
    response = readline(s);
    fprintf("Response: %s\n", response);
    if ~contains(response, "ACK")
        delete(s);
        error("Connection test failed");
    end
else
    delete(s);
    error("No response from device");
end

%% ============================================================
% MEMORY PRE-ALLOCATION
% ============================================================

all_rx            = zeros(RX_LEN, NUM_MEASUREMENTS);
all_zc_ref        = zeros(Nzc, NUM_MEASUREMENTS);
all_lags_xcorr    = zeros(NUM_MEASUREMENTS, 1);
all_lags_energy   = zeros(NUM_MEASUREMENTS, 1);
all_lags_phase    = zeros(NUM_MEASUREMENTS, 1);
all_peaks         = zeros(NUM_MEASUREMENTS, 1);
all_snr           = zeros(NUM_MEASUREMENTS, 1);
all_seeds         = zeros(NUM_MEASUREMENTS, 1);
measurement_valid = false(NUM_MEASUREMENTS, 1);

%% ============================================================
% DATA ACQUISITION LOOP
% ============================================================

fprintf("\n========================================\n");
fprintf("MULTI-METHOD LEAK DETECTION\n");
fprintf("Measurements: %d\n", NUM_MEASUREMENTS);
fprintf("========================================\n\n");

for meas = 1:NUM_MEASUREMENTS

    fprintf("--- Measurement %d / %d ---\n", meas, NUM_MEASUREMENTS);

    % Random seed ensures TX/RX synchronization
    seed = randi(2^31-1);
    u = 1;
    all_seeds(meas) = seed;

    %% ZC PARAMETER TRANSMISSION

    flush(s);
    writeline(s, sprintf("ZC %d %d %d", seed, u, Nzc));
    pause(0.3);

    if s.NumBytesAvailable > 0
        response = readline(s);
        if ~contains(response, "OK")
            fprintf("  ZC failed: %s\n", strtrim(response));
            continue;
        end
    else
        fprintf("  No ZC response\n");
        continue;
    end

    %% ZADOFF–CHU REFERENCE GENERATION

    rng(seed, 'twister');
    n = 0:Nzc-1;
    ph0 = rand() * 2*pi;

    zc_ref = sin( ...
        2*pi*ZC_CENTER_FREQ*n/FS ...
        - pi*u*n.*(n+1)/Nzc ...
        + ph0 ...
    );

    zc_ref = zc_ref(:);
    all_zc_ref(:, meas) = zc_ref;

    %% TRIGGER TRANSMISSION

    flush(s);
    writeline(s, "TX");
    pause(0.1);

    bytes_expected = RX_LEN * 2;
    timeout_start = tic;

    while s.NumBytesAvailable < bytes_expected && toc(timeout_start) < 3
        pause(0.01);
    end

    if s.NumBytesAvailable < bytes_expected
        fprintf("  Insufficient data: %d / %d bytes\n", ...
                s.NumBytesAvailable, bytes_expected);
        continue;
    end

    % Read raw ADC data
    raw = read(s, bytes_expected, "uint8");
    rx = uint16(raw(1:2:end)) + bitshift(uint16(raw(2:2:end)), 8);
    rx = double(rx);
    all_rx(:, meas) = rx;

    %% ========================================================
    % SIGNAL PRE-PROCESSING
    % ========================================================

    rx_proc = rx - mean(rx);
    zc_proc = zc_ref - mean(zc_ref);

    [b, a] = butter(3, 3000/(FS/2), 'high');
    rx_filt = filtfilt(b, a, rx_proc);
    zc_filt = filtfilt(b, a, zc_proc);

    rx_norm = rx_filt / (std(rx_filt) + eps);
    zc_norm = zc_filt / (std(zc_filt) + eps);

    %% ========================================================
    % METHOD 1: CROSS-CORRELATION
    % ========================================================

    [r, lags] = xcorr(rx_norm, zc_norm);
    r_env = abs(hilbert(r));
    r_abs = abs(r);

    min_lag = round(0.2 * FS / c_water);
    max_lag = round(2.0 * FS / c_water);

    valid_idx = abs(lags) >= min_lag & abs(lags) <= max_lag;
    r_env(~valid_idx) = 0;

    [peak_val, peak_idx] = max(r_env);
    lag_xcorr = lags(peak_idx);

    noise_floor = median(r_abs);
    snr_db = 10*log10((peak_val^2) / (noise_floor^2 + eps));

    %% ========================================================
    % METHOD 2: ENERGY ENVELOPE
    % ========================================================

    rx_env = abs(hilbert(rx_filt));
    rx_env_smooth = movmean(rx_env, 20);

    threshold = mean(rx_env_smooth) + 2*std(rx_env_smooth);
    [~, locs_energy] = findpeaks(rx_env_smooth, ...
                                'MinPeakHeight', threshold);

    if ~isempty(locs_energy)
        lag_energy = locs_energy(1) - floor(Nzc/2);
    else
        lag_energy = lag_xcorr;
    end

    %% ========================================================
    % METHOD 3: PHASE CORRELATION
    % ========================================================

    Lfft = 2^nextpow2(max(length(rx_norm), length(zc_norm)));
    RX_fft = fft(rx_norm, Lfft);
    ZC_fft = fft(zc_norm, Lfft);

    cross_spec = RX_fft .* conj(ZC_fft);
    phase_corr = abs(ifft(cross_spec ./ (abs(cross_spec) + eps)));

    search_len = min(max_lag * 2, floor(length(phase_corr)/2));
    [~, idxp] = max(phase_corr(1:search_len));

    lag_phase = idxp - 1;
    if lag_phase > search_len/2
        lag_phase = lag_phase - search_len;
    end

    %% ========================================================
    % STORE MEASUREMENT RESULTS
    % ========================================================

    all_lags_xcorr(meas)  = lag_xcorr;
    all_lags_energy(meas) = lag_energy;
    all_lags_phase(meas)  = lag_phase;
    all_peaks(meas)       = peak_val;
    all_snr(meas)         = snr_db;

    if snr_db > MIN_SNR_DB && peak_val > MIN_CORRELATION
        measurement_valid(meas) = true;
        fprintf("  [OK]  X:%d | E:%d | P:%d | SNR: %.1f dB\n", ...
                lag_xcorr, lag_energy, lag_phase, snr_db);
    else
        fprintf("  [WEAK] X:%d | E:%d | P:%d | SNR: %.1f dB\n", ...
                lag_xcorr, lag_energy, lag_phase, snr_db);
    end

    pause(0.3);
end

%% ============================================================
% FUSION + ESTIMATION
% ============================================================

fprintf("\n========================================\n");
fprintf("FUSION ANALYSIS\n");
fprintf("========================================\n\n");

num_valid = sum(measurement_valid);
fprintf("Valid measurements: %d / %d\n", num_valid, NUM_MEASUREMENTS);

if num_valid < MIN_VALID_MEASUREMENTS
    fprintf("\n[WARNING] INSUFFICIENT DATA\n");
    delete(s);
    return;
end

valid_lags_xcorr  = all_lags_xcorr(measurement_valid);
valid_lags_energy = all_lags_energy(measurement_valid);
valid_lags_phase  = all_lags_phase(measurement_valid);
valid_snr         = all_snr(measurement_valid);

bin_width = 3;

[xcorr_estimate, xcorr_weight, xcorr_count] = ...
    cluster_and_estimate(valid_lags_xcorr, valid_snr, bin_width);

[energy_estimate, energy_weight, energy_count] = ...
    cluster_and_estimate(valid_lags_energy, valid_snr, bin_width);

[phase_estimate, phase_weight, phase_count] = ...
    cluster_and_estimate(valid_lags_phase, valid_snr, bin_width);

method_scores = [
    xcorr_count  * xcorr_weight  * 1.0;
    energy_count * energy_weight * 0.8;
    phase_count  * phase_weight  * 0.6
];
method_scores = method_scores / sum(method_scores);

final_lag = ...
    xcorr_estimate  * method_scores(1) + ...
    energy_estimate * method_scores(2) + ...
    phase_estimate  * method_scores(3);

estimate_std = std([xcorr_estimate, energy_estimate, phase_estimate]);

t_delay = abs(final_lag) / FS;
dist_water = (t_delay * c_water) / (1 + ROUND_TRIP);

fprintf("\n>> LEAK LOCATION: %.3f m (%.1f cm)\n", ...
        dist_water, dist_water*100);

%% ============================================================
% SAVE RESULTS
% ============================================================

results.distance_m     = dist_water;
results.estimate_std  = estimate_std;
results.method_scores = method_scores;

save('leak_detection_results.mat', 'results');

delete(s);

fprintf("\n========================================\n");
fprintf("Analysis complete — results saved.\n");
fprintf("========================================\n");

%% ============================================================
% HELPER FUNCTION
% ============================================================

function [estimate, avg_weight, count] = ...
    cluster_and_estimate(lags, snrs, bin_width)

    lag_min = floor(min(lags)/bin_width)*bin_width;
    lag_max = ceil(max(lags)/bin_width)*bin_width;

    edges = lag_min:bin_width:(lag_max + bin_width);
    idx = discretize(lags, edges);

    for i = 1:length(edges)-1
        in_bin(i) = sum(idx == i);
        snr_avg(i) = mean(snrs(idx == i));
        lag_avg(i) = mean(lags(idx == i));
    end

    [~, best] = max(in_bin .* snr_avg);
    estimate = lag_avg(best);
    avg_weight = snr_avg(best);
    count = in_bin(best);
end
