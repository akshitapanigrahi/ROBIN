% Initialize variables
Fs = 200; % sampling rate in Hz
numReadings = 150; % window size for plotting
numLEDs = 4; % 1 red, 3 IR LEDs
voltageData = zeros(numLEDs, numReadings); % Array for all LED voltages
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
   voltageData(2, mod(i-1, numReadings)+1) = voltage(2); % IR voltage 1
   voltageData(3, mod(i-1, numReadings)+1) = voltage(3); % IR voltage 2
   voltageData(4, mod(i-1, numReadings)+1) = voltage(4); % IR voltage 3

  
  % Separate red and IR voltages
   red = voltageData(1, :);
   IR1 = voltageData(2, :);
   IR2 = voltageData(3, :);
   IR3 = voltageData(4, :);

   % Avoid divide-by-zero errors in absorbance calculations
   red(red <= 0) = 1e-6;
   IR1(IR1 <= 0) = 1e-6;
   IR2(IR2 <= 0) = 1e-6;
   IR3(IR3 <= 0) = 1e-6;
  
   % Absorbance calculations
   refVoltageRed = 1.99; % Reference red voltage
   refVoltageIR1 = 1.97; % Reference IR voltage
   refVoltageIR2 = 1.97; 
   refVoltageIR3 = 1.97; 

   redAbsorbance = log10(refVoltageRed ./ red);
   irAbsorbance1 = log10(refVoltageIR1 ./ IR1);
   irAbsorbance2 = log10(refVoltageIR2 ./ IR2);
   irAbsorbance3 = log10(refVoltageIR3 ./ IR3);

   % HbO2 and HHb calculations (make sure to change epsilons)
   epsilonO2Hb1 = 0.25; %IR1
   epsilonO2Hb2 = 0.25; %IR1
   epsilonO2Hb3 = 0.25; %IR1
   epsilonHHb = 0.39; %red
   pathLength = 0.15;
   O2Hb1 = abs(irAbsorbance1) / (epsilonO2Hb1 * pathLength);
   O2Hb2 = abs(irAbsorbance2) / (epsilonO2Hb2 * pathLength);
   O2Hb3 = abs(irAbsorbance3) / (epsilonO2Hb3 * pathLength);

   O2HbAvg = (O2Hb1 + O2Hb2 + O2Hb3) / 3;

   HHb = abs(redAbsorbance / (epsilonHHb * pathLength));

   % Weightage method (for IR wavelengthd)

   %W1 = 1;
   %W2 = 1;
   %W3 = 1;

   %O2HbWeighted = ((W1 * O2Hb1) + (W2 * O2Hb2) + (W3 * O2Hb3))/3;

   % Bandpass filter setup

   % Newborn HR range is 120-160 bpm = 2-2.7 Hz 
   % Newborn FcLow = 1.5 or 1 Hz
   % Newborn FcHigh = 4 Hz

   FcLow = 0.5; % Hz
   FcHigh = 2.5; % Hz
   [b, a] = butter(2, [FcLow FcHigh] / (Fs / 2), 'bandpass');
  
   % HbT and SpO2 calculations
   HbT = O2HbAvg + HHb;
   SpO2 = (O2HbAvg ./ HbT) * 100; % percentage
  
   % PI calculations (Perfusion Index)

   % Low-pass filter for DC (non-pulsatile) component
   [b_low, a_low] = butter(2, FcLow / (Fs / 2), 'low'); % low-pass filter
   redDC = filtfilt(b_low, a_low, red);  % DC component of red
   IRDC1 = filtfilt(b_low, a_low, IR1);
   IRDC2 = filtfilt(b_low, a_low, IR2);
   IRDC3 = filtfilt(b_low, a_low, IR3);

   IRDCAvg = (IRDC1 + IRDC2 + IRDC3) / 3;
   
   % High-pass filter for AC (pulsatile) component
   redAC = filtfilt(b, a, red);  % AC component of red
   IRAC1 = filtfilt(b, a, IR1);    % AC component of IR1
   IRAC2 = filtfilt(b, a, IR2);    % AC component of IR2
   IRAC3 = filtfilt(b, a, IR3);    % AC component of IR3

   IRACAvg = (IRAC1 + IRAC2 + IRAC3) / 3;
   
   % Perfusion Index (PI) calculation
   PI_red = (std(redAC) / mean(redDC)) * 100;  % PI for Red
   PI_IR = (std(IRACAvg) / mean(IRDCAvg)) * 100;    % PI for IR
   
   % Combine PI (average of Red and IR)
   PI_combined = (PI_red + PI_IR) / 2;
   
   % Update the PI_combined array
   PI_combined_data(mod(i-1, numReadings) + 1) = PI_combined;  % Store in buffer

   % Calculate pulse from PI

   %Find peaks in AC component of signal
   [peaks, locs] = findpeaks(IRACAvg, 'MinPeakDistance', round(Fs * 0.5));

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
