

%% Generate Zadoff-Chu Sequence
function zc = generateZadoffChu(length, root)
    % GENERATEZADOFFCHU Creates a Zadoff-Chu CAZAC sequence
    %
    % ZC(n) = exp(-j * pi * root * n * (n+1) / length)
    %
    % Inputs:
    %   length - Sequence length (should be prime for optimal properties)
    %   root   - Root index (coprime with length)
    %
    % Output:
    %   zc - Complex ZC sequence [1 x length]
    
    if nargin < 2
        root = 29;  % Default root
    end
    if nargin < 1
        length = 127;  % Default length
    end
    
    n = 0:length-1;
    phase = -pi * root * n .* (n + 1) / length;
    zc = exp(1j * phase);
end

%% Generate Gold Code
function code = generateGoldCode(length)
    % GENERATEGOLDCODE Creates a Gold spreading code
    %
    % Gold codes are formed by XORing two m-sequences
    %
    % Input:
    %   length - Code length (2^n - 1 for some integer n)
    %
    % Output:
    %   code - Bipolar Gold code (+1/-1) [1 x length]
    
    if nargin < 1
        length = 127;  % 2^7 - 1
    end
    
    % Determine register length
    n = ceil(log2(length + 1));
    
    % Generate two m-sequences with different polynomials
    % For n=7: x^7 + x^3 + 1 and x^7 + x^3 + x^2 + x + 1
    poly1 = [7, 3, 0];
    poly2 = [7, 3, 2, 1, 0];
    
    m1 = mseq(n, poly1, length);
    m2 = mseq(n, poly2, length);
    
    % Gold code = XOR of m-sequences, converted to +1/-1
    code = 2 * xor(m1, m2) - 1;
end

function seq = mseq(n, taps, len)
    % Generate m-sequence using LFSR
    register = ones(1, n);  % Initialize with all 1s
    seq = zeros(1, len);
    
    for i = 1:len
        seq(i) = register(end);
        
        % Calculate feedback
        feedback = 0;
        for t = taps
            if t > 0 && t <= n
                feedback = xor(feedback, register(n - t + 1));
            end
        end
        
        % Shift register
        register = [feedback, register(1:end-1)];
    end
end

%% Cyclic Correlation
function corr = cyclicCorrelation(signal, reference)
    % CYCLICCORRELATION Computes cyclic (circular) correlation
    %
    % Uses FFT for efficient computation:
    %   C = IFFT(FFT(x) .* conj(FFT(y)))
    %
    % Inputs:
    %   signal    - Input signal
    %   reference - Reference sequence (template)
    %
    % Output:
    %   corr - Cyclic correlation result
    
    N = length(signal);
    M = length(reference);
    
    % Zero-pad to same length (use next power of 2 for efficiency)
    L = 2^nextpow2(max(N, M));
    
    % FFT-based correlation
    X = fft(signal, L);
    Y = fft(reference, L);
    
    corr = ifft(X .* conj(Y));
    corr = corr(1:N);  % Return same length as input
end

%% Multi-Method Leak Localization
function results = leakLocalizer(signal, fs, config)
    % LEALOCALIZER Multi-method fusion for leak position estimation
    %
    % Methods:
    %   1. Cross-correlation with ZC preamble
    %   2. Energy envelope detection
    %   3. Phase correlation
    %   4. Peak fusion and validation
    %
    % Inputs:
    %   signal - Received signal (time domain)
    %   fs     - Sample rate (Hz)
    %   config - Configuration struct with fields:
    %            .zcLength, .zcRoot, .soundSpeed, .valvePositions
    %
    % Output:
    %   results - Struct with detected leaks and diagnostics
    
    % Default configuration
    if nargin < 3
        config = struct();
    end
    if ~isfield(config, 'zcLength'), config.zcLength = 127; end
    if ~isfield(config, 'zcRoot'), config.zcRoot = 29; end
    if ~isfield(config, 'soundSpeed'), config.soundSpeed = 1480; end
    if ~isfield(config, 'valvePositions'), config.valvePositions = [0.25, 0.5, 0.75, 1.0]; end
    if ~isfield(config, 'centerFreq'), config.centerFreq = 8000; end
    
    results = struct();
    results.methods = struct();
    
    %% Method 1: ZC Cross-Correlation
    zc = generateZadoffChu(config.zcLength, config.zcRoot);
    
    % Upsample ZC to signal rate
    samplesPerChip = round(fs / (config.centerFreq / 4));
    zcUpsampled = zeros(1, config.zcLength * samplesPerChip);
    for i = 1:config.zcLength
        t = ((i-1)*samplesPerChip : i*samplesPerChip-1) / fs;
        carrier = cos(2*pi*config.centerFreq*t);
        zcUpsampled((i-1)*samplesPerChip+1 : i*samplesPerChip) = real(zc(i)) * carrier;
    end
    
    % Cross-correlation
    [xcorrResult, lags] = xcorr(signal, zcUpsampled);
    xcorrResult = abs(xcorrResult(length(signal):end));  % Positive lags only
    
    results.methods.xcorr = xcorrResult;
    
    %% Method 2: Energy Envelope Detection
    % Hilbert transform for envelope
    envelope = abs(hilbert(signal));
    
    % Smooth envelope
    windowLen = round(fs * 0.001);  % 1ms window
    envelope = movmean(envelope, windowLen);
    
    results.methods.envelope = envelope;
    
    %% Method 3: Phase Correlation
    % Compute instantaneous phase
    analytic = hilbert(signal);
    phase = unwrap(angle(analytic));
    
    % Phase derivative (instantaneous frequency)
    phaseDerivative = diff(phase) * fs / (2*pi);
    
    % Detect phase discontinuities (reflections)
    phaseDiff = abs(diff(phaseDerivative));
    phaseDiff = [0, phaseDiff];  % Pad to original length
    
    results.methods.phase = phaseDiff;
    
    %% Peak Detection for Each Method
    minPeakDist = round(fs * 0.005);  % 5ms minimum between peaks
    
    % XCorr peaks
    [~, xcorrPeaks] = findpeaks(results.methods.xcorr, ...
        'MinPeakHeight', mean(results.methods.xcorr) + 3*std(results.methods.xcorr), ...
        'MinPeakDistance', minPeakDist, ...
        'SortStr', 'descend', 'NPeaks', 10);
    
    % Envelope peaks
    [~, envPeaks] = findpeaks(results.methods.envelope, ...
        'MinPeakHeight', mean(results.methods.envelope) + 3*std(results.methods.envelope), ...
        'MinPeakDistance', minPeakDist, ...
        'SortStr', 'descend', 'NPeaks', 10);
    
    % Phase peaks
    [~, phasePeaks] = findpeaks(results.methods.phase, ...
        'MinPeakHeight', mean(results.methods.phase) + 3*std(results.methods.phase), ...
        'MinPeakDistance', minPeakDist, ...
        'SortStr', 'descend', 'NPeaks', 10);
    
    %% Convert Peaks to Distances
    sampleToDistance = @(s) (s / fs * config.soundSpeed) / 2;
    
    xcorrDistances = sampleToDistance(xcorrPeaks);
    envDistances = sampleToDistance(envPeaks);
    phaseDistances = sampleToDistance(phasePeaks);
    
    results.xcorrDistances = xcorrDistances;
    results.envDistances = envDistances;
    results.phaseDistances = phaseDistances;
    
    %% Fusion: Find Consistent Detections
    tolerance = 0.03;  % 3cm tolerance
    
    leaks = [];
    for v = 1:length(config.valvePositions)
        valveDist = config.valvePositions(v);
        
        % Check each method
        xcorrHit = any(abs(xcorrDistances - valveDist) < tolerance);
        envHit = any(abs(envDistances - valveDist) < tolerance);
        phaseHit = any(abs(phaseDistances - valveDist) < tolerance);
        
        % Confidence based on agreement
        votes = xcorrHit + envHit + phaseHit;
        
        if votes >= 2  % At least 2 methods agree
            leak = struct();
            leak.position = valveDist;
            leak.valveIndex = v;
            leak.confidence = votes / 3;
            leak.methods = struct('xcorr', xcorrHit, 'envelope', envHit, 'phase', phaseHit);
            
            leaks = [leaks, leak];
        end
    end
    
    results.leaks = leaks;
    results.numLeaks = length(leaks);
end

%% SNR Estimator
function snr = snrEstimator(correlation, method)
    % SNRESTIMATOR Estimate signal-to-noise ratio from correlation
    %
    % Methods:
    %   'peak'  - SNR = peak / noise_floor
    %   'rms'   - SNR = signal_rms / noise_rms
    %   'power' - SNR = signal_power / noise_power
    %
    % Inputs:
    %   correlation - Correlation data
    %   method      - 'peak' (default), 'rms', or 'power'
    %
    % Output:
    %   snr - Estimated SNR in dB
    
    if nargin < 2
        method = 'peak';
    end
    
    % Separate signal and noise regions
    % Assume signal is in peaks, noise is in valleys
    threshold = mean(correlation);
    
    signalRegion = correlation(correlation > threshold);
    noiseRegion = correlation(correlation <= threshold);
    
    if isempty(noiseRegion)
        noiseRegion = min(correlation);
    end
    
    switch lower(method)
        case 'peak'
            signalPower = max(correlation)^2;
            noisePower = var(noiseRegion);
        case 'rms'
            signalPower = mean(signalRegion.^2);
            noisePower = mean(noiseRegion.^2);
        case 'power'
            signalPower = sum(signalRegion.^2) / length(signalRegion);
            noisePower = sum(noiseRegion.^2) / length(noiseRegion);
        otherwise
            error('Unknown method: %s', method);
    end
    
    if noisePower == 0
        snr = Inf;
    else
        snr = 10 * log10(signalPower / noisePower);
    end
end

%% Waveform Generator (for testing)
function [waveform, time] = generateTestWaveform(fs, duration, config)
    % GENERATETESTWAVEFORM Create test signal with echoes
    %
    % Generates a simulated received signal with:
    %   - Direct path (TX->RX)
    %   - Valve reflections
    %   - Pipe end reflection
    %   - AWGN noise
    %
    % Inputs:
    %   fs       - Sample rate
    %   duration - Signal duration (seconds)
    %   config   - Configuration struct
    %
    % Outputs:
    %   waveform - Simulated received signal
    %   time     - Time axis
    
    if nargin < 3
        config = struct();
    end
    if ~isfield(config, 'soundSpeed'), config.soundSpeed = 1480; end
    if ~isfield(config, 'valvePositions'), config.valvePositions = [0.25, 0.5, 0.75, 1.0]; end
    if ~isfield(config, 'leakValves'), config.leakValves = [2, 3]; end  % Valves with leaks
    if ~isfield(config, 'snr'), config.snr = 20; end  % dB
    if ~isfield(config, 'zcLength'), config.zcLength = 127; end
    if ~isfield(config, 'zcRoot'), config.zcRoot = 29; end
    if ~isfield(config, 'centerFreq'), config.centerFreq = 8000; end
    
    time = 0:1/fs:duration-1/fs;
    N = length(time);
    waveform = zeros(1, N);
    
    % Generate ZC pulse template
    zc = generateZadoffChu(config.zcLength, config.zcRoot);
    samplesPerChip = round(fs / (config.centerFreq / 4));
    
    pulse = zeros(1, config.zcLength * samplesPerChip);
    for i = 1:config.zcLength
        t = ((i-1)*samplesPerChip : i*samplesPerChip-1) / fs;
        carrier = cos(2*pi*config.centerFreq*t);
        pulse((i-1)*samplesPerChip+1 : i*samplesPerChip) = real(zc(i)) * carrier;
    end
    pulseLen = length(pulse);
    
    % Add echoes from leak valves
    for v = config.leakValves
        if v > length(config.valvePositions)
            continue;
        end
        
        roundTrip = 2 * config.valvePositions(v) / config.soundSpeed;
        delaySamples = round(roundTrip * fs);
        
        % Attenuation based on distance
        attenuation = 1 / (1 + config.valvePositions(v)^2);
        
        if delaySamples + pulseLen <= N
            waveform(delaySamples+1 : delaySamples+pulseLen) = ...
                waveform(delaySamples+1 : delaySamples+pulseLen) + attenuation * pulse;
        end
    end
    
    % Add pipe end reflection
    roundTrip = 2 * 1.0 / config.soundSpeed;  % 1m pipe
    delaySamples = round(roundTrip * fs);
    attenuation = 0.5;  % Strong reflection from closed end
    
    if delaySamples + pulseLen <= N
        waveform(delaySamples+1 : delaySamples+pulseLen) = ...
            waveform(delaySamples+1 : delaySamples+pulseLen) + attenuation * pulse;
    end
    
    % Add AWGN
    signalPower = mean(waveform.^2);
    noisePower = signalPower / (10^(config.snr/10));
    noise = sqrt(noisePower) * randn(1, N);
    waveform = waveform + noise;
end
