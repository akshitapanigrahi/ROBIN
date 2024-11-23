% Initialize variables
Fs = 200; % sampling rate in Hz
numReadings = 150; % window size for plotting
voltageData = zeros(2, numReadings); %[red, IR] voltage
PI_combined_data = zeros(1, numReadings);  % Buffer for PI
timeVector = (0:numReadings-1) / Fs;

% Communication between Arduino and Matlab
delete(serialportfind); % clear existing serial connections
arduino = serialport("COM8", 9600);
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

% Read arduino data in real-time
i = 0;
while true
   i = i + 1;
   data = readline(arduino);
   voltage = str2double(split(data, ',')); % parse data
   voltageData(1, mod(i-1, numReadings)+1) = voltage(1); % Red voltage
   voltageData(2, mod(i-1, numReadings)+1) = voltage(2); % IR voltage
  
   % Extract red and IR
   red = voltageData(1,:);
   IR = voltageData(2,:);
  
   % Prevent divide-by-zero or negative values in absorbance calculations
   red(red <= 0) = 1e-6;  % Replace zero or negative red values with a small positive value
   IR(IR <= 0) = 1e-6;    % Replace zero or negative IR values with a small positive value
  
   % Absorbance calculations
   refVoltageRed = 1.99; % Reference red voltage
   refVoltageIR = 1.97; % Reference IR voltage
   redAbsorbance = log10(refVoltageRed ./ red);
   irAbsorbance = log10(refVoltageIR ./ IR);
  
   % O2Hb and HHb calculations
   epsilonO2Hb = 0.25;
   epsilonHHb = 0.39;
   pathLength = 0.15;
   O2Hb = abs(irAbsorbance ./ (epsilonO2Hb * pathLength));
   HHb = abs(redAbsorbance ./ (epsilonHHb * pathLength));
  
   % Bandpass filter setup

   % Newborn HR range is 120-160 bpm = 2-2.7 Hz 
   % Newborn FcLow = 1.5 or 1 Hz
   % Newborn FcHigh = 4 Hz

   FcLow = 0.5; % Hz
   FcHigh = 2.5; % Hz
   [b, a] = butter(2, [FcLow FcHigh] / (Fs / 2), 'bandpass');
  
   % HbT and SpO2 calculations
   HbT = O2Hb + HHb;
   SpO2 = (O2Hb ./ HbT) * 100; % percentage
  
   % PI calculations (Perfusion Index)

   % Low-pass filter for DC (non-pulsatile) component
   [b_low, a_low] = butter(2, FcLow / (Fs / 2), 'low'); % low-pass filter
   redDC = filtfilt(b_low, a_low, red);  % DC component of red
   IRDC = filtfilt(b_low, a_low, IR);    % DC component of IR
   
   % High-pass filter for AC (pulsatile) component
   redAC = filtfilt(b, a, red);  % AC component of red
   IRAC = filtfilt(b, a, IR);    % AC component of IR
   
   % Perfusion Index (PI) calculation
   PI_red = (std(redAC) / mean(redDC)) * 100;  % PI for Red
   PI_IR = (std(IRAC) / mean(IRDC)) * 100;    % PI for IR
   
   % Combine PI (average of Red and IR)
   PI_combined = (PI_red + PI_IR) / 2;
   
   % Update the PI_combined array
   PI_combined_data(mod(i-1, numReadings) + 1) = PI_combined;  % Store in buffer

   % Calculate pulse from PI

   %Find peaks in AC component of signal
   [peaks, locs] = findpeaks(IRAC, 'MinPeakDistance', round(sampling_rate * 0.5));

   %Calculate time difference between peaks (s)
   timeDiff = diff(locs) / Fs;

   %Convert to BPM
   pulse = [0, 60 ./ timeDiff];
  
   % Update plots
   set(hHbT, 'YData', HbT*6.45); % in g/dL
   set(hSpO2, 'YData', SpO2);
   set(hPI, 'YData', PI_combined_data);  % Plot PI array
   set(hPulse, 'YData', pulse);  % Plot PI array
  
   % Update text displays
   set(hSpO2Text, 'String', ['SpO2: ', num2str(round(SpO2, 2)), ' %']);
   set(hHbTText, 'String', ['HbT: ', num2str(round(HbT*6.45, 2)), ' g/dL']);
   set(hPIText, 'String', ['PI: ', num2str(round(PI_combined, 2)), ' %']);
   set(hPulseText, 'String', ['HR: ', num2str(round(pulse, 2)), ' bpm']);

   %  Display data
   drawnow;

end

% Close serial port when done
clear arduino
