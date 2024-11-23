% Initialize variables
Fs = 200; % Sampling rate in Hz
numReadings = 150; % Window size for plotting
numLEDs = 4; % 1 red, 3 IR LEDs
voltageData = zeros(numLEDs, numReadings); % Array for all LED voltages
PI_combined_data = zeros(1, numReadings);  % Buffer for PI
timeVector = (0:numReadings-1) / Fs;

% Initialize calibration variables
skintoneFactor = 1;
initialRedAbsorbance = zeros(1, 10);
initialIRAbsorbance1 = zeros(1, 10);

% Communication between Arduino and MATLAB
delete(instrfind); % Clear existing serial connections
arduino = serial('COM8', 'BaudRate', 9600);
fopen(arduino);
disp("Reading data from Arduino...");

% Set up figures for real-time plotting
figure;

subplot(4,1,1); % subplot for HbT
hHbT = plot(timeVector, zeros(1,numReadings));
title("NIRS HbT Readings");
xlabel('Time (s)');
ylabel('HbT (g/dL)');

subplot(4,1,2); % subplot for SpO2
hSpO2 = plot(timeVector, zeros(1,numReadings));
ylim([50 120]);
title("NIRS SpO2 Readings");
xlabel('Time (s)');
ylabel('SpO2 (%)');

subplot(4,1,3); % subplot for PI
hPI = plot(timeVector, zeros(1,numReadings));
title("Perfusion Index (PI)");
xlabel('Time (s)');
ylabel('PI (%)');

subplot(4,1,4); % subplot for pulse
hPulse = plot(timeVector, zeros(1,numReadings));
title("Heart Rate");
xlabel('Time (s)');
ylabel('Heart Rate (bpm)');

hSpO2Text = uicontrol('Style', 'text', 'FontSize', 20, 'Position', [100, -10, 200, 50]);
hHbTText = uicontrol('Style', 'text', 'FontSize', 20, 'Position', [300, -10, 200, 50]);
hPIText = uicontrol('Style', 'text', 'FontSize', 20, 'Position', [500, -10, 200, 50]);
hPulseText = uicontrol('Style', 'text', 'FontSize', 20, 'Position', [700, -10, 200, 50]);

% Function for FFT-based PI and HR calculation
function [PI, HR] = calculatePIandHR_FFT(signal, Fs)
    N = length(signal);
    fftSignal = fft(signal);
    fftMagnitude = abs(fftSignal(1:N/2+1)); % Single-sided spectrum
    freqs = (0:(N/2)) * (Fs / N);

    % Find DC and remove it for HR analysis
    DC_Component = fftMagnitude(1);
    fftMagnitude(1) = 0;

    % HR frequency band (0.5–3 Hz)
    HR_Band = (freqs >= 0.5 & freqs <= 3);
    [~, idx] = max(fftMagnitude(HR_Band));
    HR_Frequency = freqs(HR_Band);
    HR_Frequency = HR_Frequency(idx);

    % Convert frequency to BPM
    HR = HR_Frequency * 60;

    % AC Component (pulsatile signal)
    AC_Component = max(fftMagnitude(HR_Band));

    % Perfusion Index
    PI = (AC_Component / DC_Component) * 100;
end

% Real-time data processing and visualization
i = 0;
while true
    i = i + 1;
    data = fgetl(arduino);
    voltage = str2double(split(data, ',')); % Parse data
    if length(voltage) ~= numLEDs
        continue; % Skip iteration if data is incomplete
    end
    
    % Update voltage buffer
    voltageData(:, mod(i-1, numReadings)+1) = voltage;
    red = voltageData(1, :);
    IR1 = voltageData(2, :);
    IR2 = voltageData(3, :);
    IR3 = voltageData(4, :);

    % Absorbance calculations
    refVoltageRed = 1.99;
    refVoltageIR = 1.97;
    redAbsorbance = log10(refVoltageRed ./ red);
    irAbsorbance1 = log10(refVoltageIR ./ IR1);
    irAbsorbance2 = log10(refVoltageIR ./ IR2);
    irAbsorbance3 = log10(refVoltageIR ./ IR3);

    % Skintone calibration
    if i <= 10
        disp('Calibrating...');
        initialRedAbsorbance(i) = mean(redAbsorbance);
        initialIRAbsorbance1(i) = mean(irAbsorbance1);
        if i == 10
            skintoneRatio = mean(initialRedAbsorbance) / mean(initialIRAbsorbance1);
            if skintoneRatio < 1.5
                skintoneFactor = 1.1;
            end
            disp('Calibration complete.');
        end
    else
        redAbsorbance = redAbsorbance * skintoneFactor;
    end

    % HbT and SpO2 calculations
    epsilonO2Hb1 = 0.25; epsilonO2Hb2 = 0.26; epsilonO2Hb3 = 0.27; epsilonHHb = 0.39;
    pathLength = 0.15;

    O2Hb1 = abs(irAbsorbance1) / (epsilonO2Hb1 * pathLength);
    O2Hb2 = abs(irAbsorbance2) / (epsilonO2Hb2 * pathLength);
    O2Hb3 = abs(irAbsorbance3) / (epsilonO2Hb3 * pathLength);
    O2HbAvg = (O2Hb1 + O2Hb2 + O2Hb3) / 3;
    HHb = abs(redAbsorbance / (epsilonHHb * pathLength));
    HbT = O2HbAvg + HHb;
    SpO2 = (O2HbAvg ./ HbT) * 100;

    % Bandpass filter setup for AC component
    [b, a] = butter(2, [0.5 3] / (Fs / 2), 'bandpass');
    redAC = filtfilt(b, a, red);
    IRACAvg = (filtfilt(b, a, IR1) + filtfilt(b, a, IR2) + filtfilt(b, a, IR3)) / 3;

    % FFT-based PI and HR
    [PI, HR] = calculatePIandHR_FFT(IRACAvg, Fs);

    % Update plots
    set(hHbT, 'YData', HbT * 6.45); % in g/dL
    set(hSpO2, 'YData', SpO2);
    set(hPI, 'YData', [PI_combined_data(2:end), PI]); % Append to rolling buffer
    set(hPulse, 'YData', HR);

    % Update text displays
    set(hSpO2Text, 'String', sprintf('SpO2: %.2f %%', SpO2));
    set(hHbTText, 'String', sprintf('HbT: %.2f g/dL', HbT * 6.45));
    set(hPIText, 'String', sprintf('PI: %.2f %%', PI));
    set(hPulseText, 'String', sprintf('HR: %.2f bpm', HR));

    % Display data
    drawnow;
end

% Close serial port
fclose(arduino);
delete(arduino);
