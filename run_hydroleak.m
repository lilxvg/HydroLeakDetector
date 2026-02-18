

clear; clc; close all;

fprintf('================================================\n');
fprintf('   HydroLeak Acoustic Leak Detection System\n');
fprintf('================================================\n\n');

%% Configuration
% Modify these if needed
COM_PORT = '';  % Leave empty for auto-detect, or set like 'COM3' or '/dev/ttyUSB0'

%% Initialize System
fprintf('Initializing...\n');

if isempty(COM_PORT)
    hl = HydroLeak();
else
    hl = HydroLeak(COM_PORT);
end

%% Connect
fprintf('\nConnecting to ESP32...\n');
if ~hl.connect()
    fprintf('\nConnection failed. Options:\n');
    fprintf('  1. Check USB connection\n');
    fprintf('  2. Verify COM port in Device Manager\n');
    fprintf('  3. Try manual port: hl = HydroLeak(''COM3''); hl.connect();\n');
    fprintf('\nRunning SIMULATION instead...\n\n');
    
    % Run simulation
    hl = HydroLeak.simulate();
    return;
end

%% Run Detection
fprintf('\nStarting leak detection...\n');
fprintf('This will perform %d averaging bursts.\n', hl.numAverages);
fprintf('Estimated time: %.1f seconds\n\n', hl.numAverages * 0.25);

input('Press ENTER to start detection cycle...', 's');

results = hl.runDetection();

%% Visualize
fprintf('\nGenerating plots...\n');
hl.plotResults();

%% Export Results
saveResults = input('\nSave results to file? (y/n): ', 's');
if strcmpi(saveResults, 'y')
    hl.exportResults();
end

%% Run Another Cycle?
while true
    another = input('\nRun another detection cycle? (y/n): ', 's');
    if strcmpi(another, 'y')
        results = hl.runDetection();
        hl.plotResults();
    else
        break;
    end
end

%% Cleanup
fprintf('\nDisconnecting...\n');
hl.disconnect();
fprintf('Done!\n');
