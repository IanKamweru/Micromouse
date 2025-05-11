%% Left Motor
%% PWM - Voltage Mapping
clc; clear;

pzopt = pzoptions;
pzopt.Grid = 'on';
pzopt.Title.FontSize = 11;
pzopt.Title.FontWeight = 'bold';
pzopt.XLabel.FontSize = 11;
pzopt.YLabel.FontSize = 11;

% raw data
pwm_values_left = [-400, -350, -300, -250, -200, -150, -100, -75, -50, -25, ...
               25,   50,   75,  100,  150,  200,  250,  300,  350,  400];

voltages_left = [-12.20, -11.77, -11.55, -10.92, -10.28, -9.08, -6.92, -4.71, -0.61, -0.18, ...
             0.21,  0.64,  3.90,  6.30,  8.71,  10.11,  10.81,  11.29,  11.73, 12.20];

% function to interpolate pwm
function pwm = interpolate(v, voltages, pwm_values)
    N = length(voltages);

    % limit pwm to [-400,400] range
    if v <= voltages(1)
        pwm = pwm_values(1);
        return;
    elseif v >= voltages(N)
        pwm = pwm_values(N);
        return;
    end

    % find interval and interpolate linearly
    for i = 1:N-1
        v1 = voltages(i);
        v2 = voltages(i+1);
        if v >= v1 && v <= v2
            pwm1 = pwm_values(i);
            pwm2 = pwm_values(i+1);
            slope = (pwm2 - pwm1) / (v2 - v1);
            pwm = pwm1 + slope * (v - v1);
            return;
        end
    end
end

v = linspace(min(voltages_left), max(voltages_left), 100);
pwm_interp = arrayfun(@(v) interpolate(v, voltages_left, pwm_values_left), v);

% plot
figure;
plot(voltages_left, pwm_values_left, 'ko', 'MarkerFaceColor', 'k'); hold on;
plot(v, pwm_interp, 'b-', 'LineWidth', 1.6);
xlabel('Voltage (V)');
ylabel('PWM Command');
title('Voltage to PWM Linear Interpolation (Left Motor)');
legend('Raw Data', 'Interpolated Curve', 'Location', 'northwest');
grid on;

%% Km and Tau Characterization
% Read the data from the saved text file
data = readtable("400.txt");

% Shift the timestamps to start at t = 0
time_s = (data.Time_us - data.Time_us(1)) / 1e6;

% Extract speed data
speed_rad_per_s = data.Speed_rad_per_s;

% Plot the speed vs time
figure;
plot(time_s, speed_rad_per_s, 'b-', 'LineWidth', 1.5);
title('\bfMotor Speed vs Time');
xlabel('Time (s)');
ylabel('Speed (rad/s)');
xlim([0 0.5]);
grid on;

%%
num_samples = length(pwm_values_left);
pwm_commands = zeros(num_samples, 1);
voltages = zeros(num_samples, 1);
Kms_per_PWM = zeros(num_samples, 1);
Kms_per_V = zeros(num_samples, 1);
taus = zeros(num_samples, 1);
steady_speeds = zeros(num_samples, 1);

% Process each file
for i = 1:num_samples
    % File name based on the PWM value
    filename = sprintf("%d_left.txt", pwm_values_left(i));
    data = readtable(filename);

    % Extract time and speed
    time_us = data.Time_us;
    speed_rad_per_s = data.Speed_rad_per_s;

    % Shift time to start at 0
    time_s = (time_us - time_us(1)) / 1e6;

    % Calculate steady speed (average of last 25% of the data)
    n_last_25_percent = round(0.25 * length(speed_rad_per_s));
    steady_speed = mean(speed_rad_per_s(end - n_last_25_percent + 1:end));

    % Calculate Tau (63.2% response time)
    if pwm_values_left(i) > 0
        target_speed = 0.632 * steady_speed;
        tau_idx = find(speed_rad_per_s <= target_speed, 1);
    else
        target_speed = 0.632 * steady_speed;
        tau_idx = find(speed_rad_per_s >= target_speed, 1);
    end

    if ~isempty(tau_idx)
        tau = time_s(tau_idx);
    else
        tau = NaN;  % If no valid point is found
    end

    % Calculate Km values
    Km_rad_per_s_per_PWM = steady_speed / pwm_values_left(i);
    Km_rad_per_s_per_V = steady_speed / voltages_left(i);

    % Store in pre-allocated arrays
    pwm_commands(i) = pwm_values_left(i);
    voltages(i) = voltages_left(i);
    Kms_per_PWM(i) = Km_rad_per_s_per_PWM;
    Kms_per_V(i) = Km_rad_per_s_per_V;
    taus(i) = tau;
    steady_speeds(i) = steady_speed;
end

% Create the results table
results = table(pwm_commands, voltages, Kms_per_PWM, Kms_per_V, taus, steady_speeds, ...
    'VariableNames', {'PWM_Command', 'Voltage_V', 'Km_rad_per_s_per_PWM', 'Km_rad_per_s_per_V', 'Tau_s', 'Steady_Speed_rad_per_s'});

% Display the results table
disp(results);

% Motor Speed vs Voltage and PWM Plot

% Extract data from the results table
voltage = results.Voltage_V;
pwm = results.PWM_Command;
steady_speed = results.Steady_Speed_rad_per_s;

% Plot Motor Speed vs Voltage
figure;
plot(voltage, steady_speed, 'o-', 'LineWidth', 1.5);
title('\bfMotor Speed vs Voltage');
xlabel('Voltage (V)');
ylabel('Steady Speed (rad/s)');
grid on;

% Plot Motor Speed vs PWM
figure;
plot(pwm, steady_speed, 'o-', 'LineWidth', 1.5);
title('\bfMotor Speed vs PWM');
xlabel('PWM Command');
ylabel('Steady Speed (rad/s)');
grid on;

%% Right Motor
