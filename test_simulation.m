clear; clc; close all;
% PHYSICAL PARAMETERS
fs           = 48000;     % Sample rate (Hz)
c_water      = 1480;      % Speed of sound in water (m/s)
pipeLength   = 1.0;       % Pipe length (m)

valvePos = [0.2.5 0.50 0.75 1.00];   % Valve locations along pipe (m)
leakValves = [2 3];                % Ground truth leaking valves


% CORRELATION AXIS SETUP
% Distance mapping assumes round-trip propagation

corrLen = 500;
corrData = zeros(1, corrLen);

distanceAxis = ((0:corrLen-1) / fs) * c_water / 2;


% NOISE FLOOR
noiseLevel = 0.05;
corrData = noiseLevel * abs(randn(1, corrLen));

% INSERT LEAK REFLECTIONS
peakWidth    = 5;     % Samples
peakStrength = 1.0;


for v = leakValves
    trueDist = valvePos(v);
    [~, idx] = min(abs(distanceAxis - trueDist));

    peak = peakStrength * exp(-((1:corrLen) - idx).^2 / (2*peakWidth^2));
    corrData = corrData + peak;
end

% Pipe end reflection
[~, idxEnd] = min(abs(distanceAxis - pipeLength));


endPeak = 0.8 * exp(-((1:corrLen) - idxEnd).^2 / (2*peakWidth^2));
corrData = corrData + endPeak;

% PEAK DETECTION
threshold = mean(corrData) + 1.5 * std(corrData);


[pVals, pLocs] = findpeaks(corrData, ...
    'MinPeakHeight', threshold, ...
    'MinPeakDistance', 3);



% PEAK-TO-VALVE ASSOCIATION
tolerance = 0.05;   % meters
detectedLeaks = [];

fprintf('Peak analysis:\n');

for i = 1:numel(pLocs)
    d = distanceAxis(pLocs(i));
    fprintf('  Peak %d → %.3f m (%.3f)\n', i, d, pVals(i));

    for v = 1:numel(valvePos)
        if abs(d - valvePos(v)) < tolerance
            fprintf('     MATCH: V%d (%.2f m)\n', v, valvePos(v));
            detectedLeaks(end+1) = v; %#ok<SAGROW>
        end
    end
end



% RESULTS SUMMARY


if isempty(detectedLeaks)
    fprintf('  NO LEAKS DETECTED\n');
else
    fprintf('  LEAKS DETECTED AT:\n');
    for v = detectedLeaks
        fprintf('    V%d = %.2f m\n', v, valvePos(v));
    end
end


% VISUALIZATION
figure('Name','HydroLeak Simulation','Position',[100 100 1200 500]);

subplot(1,2,1);
plot(distanceAxis, corrData, 'b', 'LineWidth', 1.5); hold on;

for v = 1:numel(valvePos)
    xline(valvePos(v), '--r', sprintf('V%d', v), 'LineWidth', 1.2);
end

plot(distanceAxis(pLocs), pVals, 'go', ...
    'MarkerSize', 12, 'MarkerFaceColor','g');

yline(threshold, ':k', 'Threshold');

xlabel('Distance (m)');
ylabel('Correlation Magnitude');
title('Correlation vs Distance');
xlim([0 1.2]);
grid on;

subplot(1,2,2); hold on;

rectangle('Position',[0 0.3 1 0.4], ...
    'FaceColor',[0.8 0.9 1], 'EdgeColor','k', 'LineWidth',2);

rectangle('Position',[-0.05 0.65 0.1 0.1], ...
    'FaceColor','b', 'EdgeColor','k', 'Curvature',0.3);
text(0,0.82,'Sensor','HorizontalAlignment','center','FontWeight','bold');

for v = 1:numel(valvePos)
    if ismember(v, detectedLeaks)
        col = 'r';
    else
        col = [0.7 0.7 0.7];
    end

    rectangle('Position',[valvePos(v)-0.03 0.2 0.06 0.6], ...
        'FaceColor',col,'EdgeColor','k','LineWidth',1.5);

    text(valvePos(v),0.05, ...
        sprintf('V%d\n%.2fm',v,valvePos(v)), ...
        'HorizontalAlignment','center');
end

xlim([-0.15 1.15]);
ylim([0 1]);
axis off;
title('Pipe Schematic (RED = Leak)');

sgtitle('HydroLeak Correlation-Domain Validation','FontWeight','bold');

fprintf('Simulation complete — inspect figure.\n');
