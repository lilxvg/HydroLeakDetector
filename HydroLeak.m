

classdef HydroLeak < handle
    properties
        % Serial connection
        serialPort
        portName
        baudRate = 921600
        
        % System configuration (synced with ESP32)
        sampleRate = 48000
        zcLength = 127
        zcRoot = 29
        goldLength = 127
        numAverages = 32
        pipeLength = 1.0       % meters
        soundSpeed = 1480      % m/s in water
        valvePositions = [0.25, 0.50, 0.75, 1.00]  % meters
        
        % Data storage
        rawData
        correlationData
        peakData
        leakResults
        txWaveform
        zcSequence
        goldCode
        
        % Processing results
        detectedLeaks
        snrEstimate
        
        % Status
        isConnected = false
        lastError = ''
    end
    
    methods
        %% Constructor
        function obj = HydroLeak(port)
            if nargin < 1
                % Auto-detect port
                port = obj.detectPort();
            end
            obj.portName = port;
        end
        
        %% Port Detection
        function port = detectPort(obj)
            % Try common port names
            if ispc
                ports = {'COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8'};
            elseif ismac
                ports = {'/dev/tty.usbmodem*', '/dev/tty.usbserial*'};
            else
                ports = {'/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1'};
            end
            
            for i = 1:length(ports)
                try
                    if contains(ports{i}, '*')
                        % Wildcard - find matching
                        d = dir(ports{i});
                        if ~isempty(d)
                            port = fullfile(d(1).folder, d(1).name);
                            return;
                        end
                    else
                        % Direct test
                        s = serialport(ports{i}, obj.baudRate);
                        delete(s);
                        port = ports{i};
                        return;
                    end
                catch
                    continue;
                end
            end
            port = '';
            warning('No serial port detected. Please specify manually.');
        end
        
        %% Connect to ESP32
        function success = connect(obj)
            try
                fprintf('Connecting to %s at %d baud...\n', obj.portName, obj.baudRate);
                
                % Close any existing connection
                obj.disconnect();
                
                % Create new connection
                obj.serialPort = serialport(obj.portName, obj.baudRate);
                configureTerminator(obj.serialPort, 'LF');
                obj.serialPort.Timeout = 10;
                
                % Flush buffers
                flush(obj.serialPort);
                pause(0.5);
                
                % Test connection
                writeline(obj.serialPort, 'PING');
                response = readline(obj.serialPort);
                
                if contains(response, 'PONG')
                    obj.isConnected = true;
                    fprintf('Connected successfully!\n');
                    
                    % Sync configuration
                    obj.syncConfig();
                    success = true;
                else
                    error('Invalid response from ESP32');
                end
                
            catch ME
                obj.lastError = ME.message;
                fprintf('Connection failed: %s\n', ME.message);
                success = false;
            end
        end
        
        %% Disconnect
        function disconnect(obj)
            if ~isempty(obj.serialPort) && isvalid(obj.serialPort)
                delete(obj.serialPort);
            end
            obj.serialPort = [];
            obj.isConnected = false;
        end
        
        %% Sync Configuration from ESP32
        function syncConfig(obj)
            fprintf('Syncing configuration...\n');
            
            writeline(obj.serialPort, 'CONFIG');
            
            config = {};
            while true
                line = readline(obj.serialPort);
                if contains(line, '</CONFIG>')
                    break;
                end
                config{end+1} = char(line);
            end
            
            % Parse configuration
            for i = 1:length(config)
                line = config{i};
                if contains(line, '=')
                    parts = strsplit(line, '=');
                    key = strtrim(parts{1});
                    value = str2double(parts{2});
                    
                    switch key
                        case 'SAMPLE_RATE'
                            obj.sampleRate = value;
                        case 'ZC_LENGTH'
                            obj.zcLength = value;
                        case 'ZC_ROOT'
                            obj.zcRoot = value;
                        case 'GOLD_LENGTH'
                            obj.goldLength = value;
                        case 'NUM_AVERAGES'
                            obj.numAverages = value;
                        case 'PIPE_LENGTH'
                            obj.pipeLength = value;
                        case 'SOUND_SPEED'
                            obj.soundSpeed = value;
                    end
                end
            end
            
            fprintf('Configuration synced:\n');
            fprintf('  Sample Rate: %d Hz\n', obj.sampleRate);
            fprintf('  ZC Length: %d, Root: %d\n', obj.zcLength, obj.zcRoot);
            fprintf('  Gold Length: %d\n', obj.goldLength);
            fprintf('  Averaging: %d bursts\n', obj.numAverages);
            fprintf('  Pipe: %.2f m, Sound Speed: %.1f m/s\n', obj.pipeLength, obj.soundSpeed);
        end
        
        %% Run Detection Cycle
        function results = runDetection(obj)
            if ~obj.isConnected
                error('Not connected to ESP32');
            end
            
            fprintf('\n========================================\n');
            fprintf('Starting Leak Detection Cycle\n');
            fprintf('========================================\n\n');
            
            % Clear previous data
            obj.rawData = [];
            obj.correlationData = [];
            obj.peakData = [];
            obj.leakResults = [];
            
            % Send start command
            writeline(obj.serialPort, 'START');
            
            % Wait for acknowledgment
            response = readline(obj.serialPort);
            if ~contains(response, 'ACK:START')
                error('ESP32 did not acknowledge START command');
            end
            
            fprintf('Detection in progress...\n');
            
            % Read data stream
            dataStarted = false;
            currentSection = '';
            dataBuffer = '';
            
            tic;
            while true
                if obj.serialPort.NumBytesAvailable > 0
                    line = char(readline(obj.serialPort));
                    
                    if contains(line, '<DATA_START>')
                        dataStarted = true;
                        fprintf('Receiving data...\n');
                    elseif contains(line, '<DATA_END>')
                        fprintf('Data reception complete (%.1f seconds)\n', toc);
                        break;
                    elseif dataStarted
                        % Parse sections
                        if contains(line, '<CORRELATION>')
                            currentSection = 'CORRELATION';
                            dataBuffer = '';
                        elseif contains(line, '</CORRELATION>')
                            obj.correlationData = str2num(dataBuffer);
                            fprintf('  Correlation: %d points\n', length(obj.correlationData));
                        elseif contains(line, '<PEAKS>')
                            currentSection = 'PEAKS';
                        elseif contains(line, '</PEAKS>')
                            currentSection = '';
                        elseif contains(line, '<LEAKS>')
                            currentSection = 'LEAKS';
                        elseif contains(line, '</LEAKS>')
                            currentSection = '';
                        elseif contains(line, '<RAW_RX>')
                            currentSection = 'RAW_RX';
                            dataBuffer = '';
                        elseif contains(line, '</RAW_RX>')
                            obj.rawData = str2num(dataBuffer);
                            fprintf('  Raw samples: %d\n', length(obj.rawData));
                        else
                            % Accumulate data
                            switch currentSection
                                case 'CORRELATION'
                                    dataBuffer = [dataBuffer, line];
                                case 'PEAKS'
                                    if ~isempty(strtrim(line))
                                        parts = str2double(strsplit(line, ','));
                                        if length(parts) >= 3
                                            obj.peakData = [obj.peakData; parts(1:3)];
                                        end
                                    end
                                case 'LEAKS'
                                    if contains(line, 'NUM_LEAKS')
                                        % Parse number of leaks
                                    elseif contains(line, 'LEAK')
                                        parts = regexp(line, 'LEAK\d+=([0-9.]+),([0-9.]+)', 'tokens');
                                        if ~isempty(parts)
                                            obj.leakResults = [obj.leakResults; ...
                                                str2double(parts{1}{1}), str2double(parts{1}{2})];
                                        end
                                    end
                                case 'RAW_RX'
                                    dataBuffer = [dataBuffer, line];
                            end
                        end
                    end
                else
                    pause(0.01);
                end
                
                % Timeout check
                if toc > 60
                    warning('Detection timeout after 60 seconds');
                    break;
                end
            end
            
            % Process and analyze results
            results = obj.analyzeResults();
            obj.detectedLeaks = results;
            
            % Display summary
            obj.displayResults(results);
        end
        
        %% Analyze Results with Multi-Method Fusion
        function results = analyzeResults(obj)
            results = struct();
            results.timestamp = datetime('now');
            results.leaks = [];
            
            if isempty(obj.correlationData)
                warning('No correlation data received');
                return;
            end
            
            % Time/distance axis
            downsample = 10;  % ESP32 downsamples by 10
            numSamples = length(obj.correlationData);
            sampleIndices = (0:numSamples-1) * downsample;
            timeAxis = sampleIndices / obj.sampleRate;
            distanceAxis = (timeAxis * obj.soundSpeed) / 2;  % One-way distance
            
            results.timeAxis = timeAxis;
            results.distanceAxis = distanceAxis;
            results.correlation = obj.correlationData;
            
            % Method 1: ESP32 Peak Detection
            if ~isempty(obj.peakData)
                results.esp32Peaks = obj.peakData;
            end
            
            % Method 2: MATLAB Enhanced Peak Detection
            % Use lower threshold and ensure MinPeakDistance is valid
            threshold = mean(obj.correlationData) + 2*std(obj.correlationData);
            
            % MinPeakDistance must be less than array length - 1
            maxValidDist = length(obj.correlationData) - 2;
            minDistSamples = min(5, maxValidDist);  % Use small value, max 5 samples apart
            
            [matlabPeaks, matlabLocs] = findpeaks(obj.correlationData, ...
                'MinPeakHeight', threshold, ...
                'MinPeakDistance', max(1, minDistSamples), ...
                'SortStr', 'descend');
            
            results.matlabPeaks = [];
            for i = 1:min(10, length(matlabLocs))
                idx = matlabLocs(i);
                dist = distanceAxis(idx);
                results.matlabPeaks = [results.matlabPeaks; idx, matlabPeaks(i), dist];
            end
            
            % Method 3: Cross-Correlation Enhancement
            if ~isempty(obj.rawData)
                % Generate local ZC reference
                zcRef = obj.generateZCSequence();
                
                % Cross-correlate (simplified - full would need matched filter)
                xcorrResult = xcorr(obj.rawData, zcRef(1:min(500, length(zcRef))));
                xcorrResult = abs(xcorrResult(length(obj.rawData):end));
                
                results.xcorrEnhanced = xcorrResult;
            end
            
            % Fuse results - find consistent peaks across methods
            tolerance = 0.03;  % 3cm tolerance
            
            fusedLeaks = [];
            for v = 1:length(obj.valvePositions)
                valveDist = obj.valvePositions(v);
                
                % Check ESP32 results
                esp32Match = false;
                esp32Strength = 0;
                if ~isempty(obj.peakData)
                    for p = 1:size(obj.peakData, 1)
                        if abs(obj.peakData(p, 3) - valveDist) < tolerance
                            esp32Match = true;
                            esp32Strength = obj.peakData(p, 2);
                            break;
                        end
                    end
                end
                
                % Check MATLAB results
                matlabMatch = false;
                matlabStrength = 0;
                if ~isempty(results.matlabPeaks)
                    for p = 1:size(results.matlabPeaks, 1)
                        if abs(results.matlabPeaks(p, 3) - valveDist) < tolerance
                            matlabMatch = true;
                            matlabStrength = results.matlabPeaks(p, 2);
                            break;
                        end
                    end
                end
                
                % Fusion decision
                if esp32Match || matlabMatch
                    confidence = 0;
                    if esp32Match, confidence = confidence + 0.5; end
                    if matlabMatch, confidence = confidence + 0.5; end
                    
                    avgStrength = mean([esp32Strength, matlabStrength]);
                    
                    fusedLeaks = [fusedLeaks; struct(...
                        'position', valveDist, ...
                        'confidence', confidence, ...
                        'strength', avgStrength, ...
                        'valveIndex', v)];
                end
            end
            
            results.leaks = fusedLeaks;
            
            % Calculate SNR estimate
            if ~isempty(obj.correlationData)
                signal = max(obj.correlationData);
                noise = std(obj.correlationData(obj.correlationData < mean(obj.correlationData)));
                results.snr = 20 * log10(signal / noise);
            else
                results.snr = NaN;
            end
        end
        
        %% Display Results
        function displayResults(obj, results)
            fprintf('\n========================================\n');
            fprintf('LEAK DETECTION RESULTS\n');
            fprintf('========================================\n');
            fprintf('Time: %s\n', datestr(results.timestamp));
            fprintf('SNR Estimate: %.1f dB\n\n', results.snr);
            
            if isempty(results.leaks)
                fprintf('No leaks detected.\n');
            else
                fprintf('Detected Leaks:\n');
                fprintf('%-10s %-12s %-12s %-10s\n', 'Valve', 'Distance', 'Confidence', 'Strength');
                fprintf('%-10s %-12s %-12s %-10s\n', '-----', '--------', '----------', '--------');
                
                for i = 1:length(results.leaks)
                    leak = results.leaks(i);
                    fprintf('V%d         %.2f m      %.0f%%          %.2f\n', ...
                        leak.valveIndex, leak.position, leak.confidence*100, leak.strength);
                end
            end
            fprintf('========================================\n\n');
        end
        
        %% Visualization
        function plotResults(obj)
            if isempty(obj.correlationData)
                warning('No data to plot');
                return;
            end
            
            figure('Name', 'HydroLeak Detection Results', ...
                   'Position', [100 100 1400 900], ...
                   'Color', 'w');
            
            % Time/distance axis
            downsample = 10;
            numSamples = length(obj.correlationData);
            sampleIndices = (0:numSamples-1) * downsample;
            timeAxis = sampleIndices / obj.sampleRate * 1000;  % ms
            distanceAxis = (sampleIndices / obj.sampleRate * obj.soundSpeed) / 2;
            
            % Plot 1: Correlation vs Distance
            subplot(2,2,1);
            plot(distanceAxis, obj.correlationData, 'b-', 'LineWidth', 1);
            hold on;
            
            % Mark valve positions
            for v = 1:length(obj.valvePositions)
                xline(obj.valvePositions(v), '--r', sprintf('V%d', v), ...
                    'LineWidth', 1.5, 'LabelOrientation', 'horizontal');
            end
            
            % Mark detected leaks
            if ~isempty(obj.detectedLeaks) && ~isempty(obj.detectedLeaks.leaks)
                for i = 1:length(obj.detectedLeaks.leaks)
                    leak = obj.detectedLeaks.leaks(i);
                    [~, idx] = min(abs(distanceAxis - leak.position));
                    plot(distanceAxis(idx), obj.correlationData(idx), 'go', ...
                        'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
                end
            end
            
            xlabel('Distance from Sensor (m)');
            ylabel('Correlation Magnitude');
            title('Correlation vs Distance');
            xlim([0 max(obj.valvePositions) + 0.2]);
            grid on;
            legend('Correlation', 'Valve Positions', 'Location', 'best');
            
            % Plot 2: Correlation vs Time
            subplot(2,2,2);
            plot(timeAxis, obj.correlationData, 'b-', 'LineWidth', 1);
            xlabel('Time (ms)');
            ylabel('Correlation Magnitude');
            title('Correlation vs Time');
            grid on;
            
            % Plot 3: Raw Waveform
            subplot(2,2,3);
            if ~isempty(obj.rawData)
                rawTime = (0:length(obj.rawData)-1) / obj.sampleRate * 10 * 1000;  % ms (downsampled)
                plot(rawTime, obj.rawData, 'k-', 'LineWidth', 0.5);
                xlabel('Time (ms)');
                ylabel('Amplitude');
                title('Raw Received Signal');
                grid on;
            else
                text(0.5, 0.5, 'No raw data', 'HorizontalAlignment', 'center');
            end
            
            % Plot 4: Detection Summary
            subplot(2,2,4);
            hold on;
            
            % Draw pipe schematic
            rectangle('Position', [0, 0.3, 1, 0.4], 'FaceColor', [0.8 0.8 0.9], ...
                'EdgeColor', 'k', 'LineWidth', 2);
            
            % Draw sensor at top
            rectangle('Position', [-0.05, 0.65, 0.1, 0.1], 'FaceColor', 'b', ...
                'EdgeColor', 'k', 'Curvature', 0.3);
            text(0, 0.8, 'Sensor', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
            
            % Draw valves
            valveColors = {'g', 'g', 'g', 'g'};  % Default: no leak (green)
            
            if ~isempty(obj.detectedLeaks) && ~isempty(obj.detectedLeaks.leaks)
                for i = 1:length(obj.detectedLeaks.leaks)
                    leak = obj.detectedLeaks.leaks(i);
                    valveColors{leak.valveIndex} = 'r';  % Leak detected (red)
                end
            end
            
            for v = 1:length(obj.valvePositions)
                pos = obj.valvePositions(v);
                rectangle('Position', [pos-0.03, 0.2, 0.06, 0.6], ...
                    'FaceColor', valveColors{v}, 'EdgeColor', 'k', 'LineWidth', 1.5);
                text(pos, 0.1, sprintf('V%d\n%.2fm', v, pos), ...
                    'HorizontalAlignment', 'center', 'FontSize', 9);
            end
            
            xlim([-0.1 1.2]);
            ylim([0 1]);
            axis off;
            title('Pipe Schematic (Red = Leak Detected)');
            
            % Add results text
            if ~isempty(obj.detectedLeaks)
                str1 = sprintf('SNR: %.1f dB', obj.detectedLeaks.snr);
                if ~isempty(obj.detectedLeaks.leaks)
                    str2 = sprintf('Leaks: %d detected', length(obj.detectedLeaks.leaks));
                else
                    str2 = 'No leaks detected';
                end
                text(0.5, 0.98, str1, 'HorizontalAlignment', 'center', ...
                    'FontSize', 11, 'FontWeight', 'bold');
                text(0.5, 0.92, str2, 'HorizontalAlignment', 'center', ...
                    'FontSize', 11, 'FontWeight', 'bold');
            end
            
            sgtitle('HydroLeak Acoustic Detection System', 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        %% Generate ZC Sequence (for local processing)
        function zc = generateZCSequence(obj)
            n = 0:obj.zcLength-1;
            phase = -pi * obj.zcRoot * n .* (n+1) / obj.zcLength;
            zc = exp(1j * phase);
        end
        
        %% Get TX Waveform from ESP32
        function getTxWaveform(obj)
            if ~obj.isConnected
                error('Not connected');
            end
            
            writeline(obj.serialPort, 'TX_WAVEFORM');
            
            dataBuffer = '';
            while true
                line = char(readline(obj.serialPort));
                if contains(line, '</TX_WAVEFORM>')
                    break;
                elseif ~contains(line, '<TX_WAVEFORM>')
                    dataBuffer = [dataBuffer, line];
                end
            end
            
            obj.txWaveform = str2num(dataBuffer);
            fprintf('Received TX waveform: %d samples\n', length(obj.txWaveform));
        end
        
        %% Export Results
        function exportResults(obj, filename)
            if nargin < 2
                filename = sprintf('hydroleak_results_%s.mat', ...
                    datestr(now, 'yyyy-mm-dd_HH-MM-SS'));
            end
            
            results = struct();
            results.correlationData = obj.correlationData;
            results.rawData = obj.rawData;
            results.peakData = obj.peakData;
            results.leakResults = obj.leakResults;
            results.detectedLeaks = obj.detectedLeaks;
            results.config = struct(...
                'sampleRate', obj.sampleRate, ...
                'zcLength', obj.zcLength, ...
                'zcRoot', obj.zcRoot, ...
                'goldLength', obj.goldLength, ...
                'numAverages', obj.numAverages, ...
                'pipeLength', obj.pipeLength, ...
                'soundSpeed', obj.soundSpeed, ...
                'valvePositions', obj.valvePositions);
            results.timestamp = datetime('now');
            
            save(filename, 'results');
            fprintf('Results exported to: %s\n', filename);
        end
        
        %% Destructor
        function delete(obj)
            obj.disconnect();
        end
    end
    
    %% Static Methods
    methods (Static)
        %% Quick Start
        function hl = quickStart(port)
            if nargin < 1
                port = '';
            end
            
            hl = HydroLeak(port);
            
            if hl.connect()
                results = hl.runDetection();
                hl.plotResults();
            end
        end
        
        %% Simulation Mode (no hardware)
        function hl = simulate()
            hl = HydroLeak('');
            hl.isConnected = false;
            
            fprintf('Running in SIMULATION mode\n');
            fprintf('Simulating LEAKS at V2 (0.50m) and V3 (0.75m)\n\n');
            
            % In the real system, ESP32 downsamples by 10 before sending
            % So MATLAB receives data that's already downsampled
            % The analyzeResults function accounts for this with:
            %   sampleIndices = (0:numSamples-1) * downsample
            %
            % For distance D, the array index should be:
            %   arrayIdx = D * 2 / soundSpeed * fs / downsample
            
            fs = 48000;
            soundSpeed = 1480;
            downsample = 10;
            
            valveDistances = [0.25, 0.5, 0.75, 1.0];
            
            % Calculate array indices for each distance
            % arrayIdx = D * 2 * fs / (soundSpeed * downsample)
            fprintf('Distance to array index mapping:\n');
            for i = 1:4
                idx = valveDistances(i) * 2 * fs / (soundSpeed * downsample);
                fprintf('  %.2fm -> array index %.1f\n', valveDistances(i), idx);
            end
            fprintf('\n');
            
            % The issue: valve positions map to array indices 1.6, 3.2, 4.9, 6.5
            % These are TOO CLOSE together with downsample=10
            % 
            % SOLUTION: Don't apply downsample in simulation - simulate raw data
            % then the indices will be: 16, 32, 49, 65 - much better separation
            
            % Create correlation data WITHOUT downsample applied
            % (simulating what ESP32 would see before downsampling)
            maxDist = 1.2;
            corrLen = round(maxDist * 2 * fs / soundSpeed) + 100;  % No downsample
            
            fprintf('Creating correlation array of length %d\n', corrLen);
            
            % Start with low noise floor
            hl.correlationData = 0.02 * abs(randn(1, corrLen));
            
            % Add peaks at leak positions
            leakValves = [2, 3];
            peakWidth = 10;  % samples
            
            for v = leakValves
                dist = valveDistances(v);
                % Array index without downsample
                sampleIdx = round(dist * 2 * fs / soundSpeed);
                
                fprintf('  V%d LEAK at %.2fm -> sample %d\n', v, dist, sampleIdx);
                
                if sampleIdx > 0 && sampleIdx <= corrLen
                    peakStrength = 1.0;
                    gaussian = peakStrength * exp(-((1:corrLen) - sampleIdx).^2 / (2*peakWidth^2));
                    hl.correlationData = hl.correlationData + gaussian;
                end
            end
            
            % Add pipe end at 1.0m
            dist = 1.0;
            sampleIdx = round(dist * 2 * fs / soundSpeed);
            fprintf('  V4 PIPE END at %.2fm -> sample %d\n', dist, sampleIdx);
            if sampleIdx > 0 && sampleIdx <= corrLen
                gaussian = 0.8 * exp(-((1:corrLen) - sampleIdx).^2 / (2*peakWidth^2));
                hl.correlationData = hl.correlationData + gaussian;
            end
            
            % Now DOWNSAMPLE to simulate what ESP32 sends to MATLAB
            hl.correlationData = hl.correlationData(1:downsample:end);
            
            fprintf('\nAfter downsampling: length = %d\n', length(hl.correlationData));
            
            % Generate synthetic raw data
            hl.rawData = 0.05 * randn(1, 1000);
            
            fprintf('\nCorrelation stats:\n');
            fprintf('  Min: %.4f, Max: %.4f, Mean: %.4f\n', ...
                min(hl.correlationData), max(hl.correlationData), mean(hl.correlationData));
            
            % Analyze
            results = hl.analyzeResults();
            hl.detectedLeaks = results;
            
            % Show detected peaks
            fprintf('\nDetected peaks:\n');
            if ~isempty(results.matlabPeaks)
                for i = 1:size(results.matlabPeaks, 1)
                    fprintf('  Peak %d: index=%d, strength=%.3f, distance=%.3fm\n', ...
                        i, results.matlabPeaks(i,1), results.matlabPeaks(i,2), results.matlabPeaks(i,3));
                end
            else
                fprintf('  None!\n');
            end
            fprintf('\n');
            
            hl.displayResults(results);
            hl.plotResults();
        end
    end
end
