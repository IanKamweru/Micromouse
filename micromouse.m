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
pwm_values = [-400, -350, -300, -250, -200, -150, -100, -75, -50, -25, ...
               25,   50,   75,  100,  150,  200,  250,  300,  350,  400];

voltages_left = [-12.20, -11.77, -11.55, -10.92, -10.28, -9.08, -6.92, -4.71, -0.61, -0.18, ...
             0.21,  0.64,  3.90,  6.30,  8.71,  10.11,  10.81,  11.29,  11.73, 12.20];

voltages_right = [-12.00, -11.60, -11.27, -10.81, -10.14, -9.05, -6.77, -4.64, -0.57, -0.18, ...
             0.19,  0.57,  3.74,  6.33,  8.72,  9.93,  10.74,  11.26,  11.59, 12.05];

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

% Generate interpolation points
v_left = linspace(min(voltages_left), max(voltages_left), 100);
pwm_interp_left = arrayfun(@(v) interpolate(v, voltages_left, pwm_values), v_left);

v_right = linspace(min(voltages_right), max(voltages_right), 100);
pwm_interp_right = arrayfun(@(v) interpolate(v, voltages_right, pwm_values), v_right);

% Plot both on the same figure
figure;
plot(voltages_left, pwm_values, 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'Left Raw Data'); hold on;
plot(v_left, pwm_interp_left, 'b-', 'LineWidth', 1.6, 'DisplayName', 'Left Interpolated');
plot(voltages_right, pwm_values, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Right Raw Data');
plot(v_right, pwm_interp_right, 'r-', 'LineWidth', 1.6, 'DisplayName', 'Right Interpolated');
xlabel('Voltage (V)');
ylabel('PWM Command');
title('Voltage to PWM Linear Interpolation (Left and Right Motors)');
legend('Location', 'northwest');
grid on;
hold off;

%% Km and Tau Characterization
% Initialize data arrays
num_samples = length(pwm_values);
pwm_commands_left = zeros(num_samples, 1);
voltages_left_arr = zeros(num_samples, 1);
Kms_per_PWM_left = zeros(num_samples, 1);
Kms_per_V_left = zeros(num_samples, 1);
taus_left = zeros(num_samples, 1);
steady_speeds_left = zeros(num_samples, 1);

pwm_commands_right = zeros(num_samples, 1);
voltages_right_arr = zeros(num_samples, 1);
Kms_per_PWM_right = zeros(num_samples, 1);
Kms_per_V_right = zeros(num_samples, 1);
taus_right = zeros(num_samples, 1);
steady_speeds_right = zeros(num_samples, 1);

% Process each file
subfolder = "./data";
for i = 1:num_samples
    % Left Motor
    filename_left = sprintf("%s/%d_left.txt", subfolder, pwm_values(i));
    if isfile(filename_left)
        data_left = readtable(filename_left);
        time_us_left = data_left.Time_us;
        speed_rad_per_s_left = data_left.Speed_rad_per_s;
        time_s_left = (time_us_left - time_us_left(1)) / 1e6;

        % Steady speed (last 25%)
        n_last_25_percent_left = round(0.25 * length(speed_rad_per_s_left));
        steady_speed_left = mean(speed_rad_per_s_left(end - n_last_25_percent_left + 1:end));

        % Tau calculation
        if pwm_values(i) > 0
            target_speed_left = 0.632 * steady_speed_left;
            tau_idx_left = find(speed_rad_per_s_left <= target_speed_left, 1);
        else
            target_speed_left = 0.632 * steady_speed_left;
            tau_idx_left = find(speed_rad_per_s_left >= target_speed_left, 1);
        end
        
        if ~isempty(tau_idx_left)
            tau_left = time_s_left(tau_idx_left);
        else
            tau_left = NaN;
        end

        % Store left motor results
        pwm_commands_left(i) = pwm_values(i);
        voltages_left_arr(i) = voltages_left(i);
        Kms_per_PWM_left(i) = steady_speed_left / pwm_values(i);
        Kms_per_V_left(i) = steady_speed_left / voltages_left(i);
        taus_left(i) = tau_left;
        steady_speeds_left(i) = steady_speed_left;
    end

    % Right Motor
    filename_right = sprintf("%s/%d_right.txt", subfolder, pwm_values(i));
    if isfile(filename_right)
        data_right = readtable(filename_right);
        time_us_right = data_right.Time_us;
        speed_rad_per_s_right = data_right.Speed_rad_per_s;
        time_s_right = (time_us_right - time_us_right(1)) / 1e6;

        % Steady speed (last 25%)
        n_last_25_percent_right = round(0.25 * length(speed_rad_per_s_right));
        steady_speed_right = mean(speed_rad_per_s_right(end - n_last_25_percent_right + 1:end));

        % Tau calculation
        if pwm_values(i) > 0
            target_speed_right = 0.632 * steady_speed_right;
            tau_idx_right = find(speed_rad_per_s_right <= target_speed_right, 1);
        else
            target_speed_right = 0.632 * steady_speed_right;
            tau_idx_right = find(speed_rad_per_s_right >= target_speed_right, 1);
        end
        
        if ~isempty(tau_idx_right)
            tau_right = time_s_right(tau_idx_right);
        else
            tau_right = NaN;
        end

        % Store right motor results
        pwm_commands_right(i) = pwm_values(i);
        voltages_right_arr(i) = voltages_right(i);
        Kms_per_PWM_right(i) = steady_speed_right / pwm_values(i);
        Kms_per_V_right(i) = steady_speed_right / voltages_right(i);
        taus_right(i) = tau_right;
        steady_speeds_right(i) = steady_speed_right;
    end
end

% Create the results tables
results_left = table(pwm_commands_left, voltages_left_arr, Kms_per_PWM_left, Kms_per_V_left, taus_left, steady_speeds_left, ...
    'VariableNames', {'PWM_Command', 'Voltage_V', 'Km_rad_per_s_per_PWM', 'Km_rad_per_s_per_V', 'Tau_s', 'Steady_Speed_rad_per_s'});

results_right = table(pwm_commands_right, voltages_right_arr, Kms_per_PWM_right, Kms_per_V_right, taus_right, steady_speeds_right, ...
    'VariableNames', {'PWM_Command', 'Voltage_V', 'Km_rad_per_s_per_PWM', 'Km_rad_per_s_per_V', 'Tau_s', 'Steady_Speed_rad_per_s'});

% Display the results tables
disp("Left Motor Results:");
disp(results_left);

disp("Right Motor Results:");
disp(results_right);

% Save the tables to files
writetable(results_left, 'results_left.csv');
writetable(results_right, 'results_right.csv');

% Plot Motor Speed vs Voltage (Left and Right)
figure;
plot(results_left.Voltage_V, results_left.Steady_Speed_rad_per_s, 'o-', 'LineWidth', 1.5, 'DisplayName', 'Left Motor');
hold on;
plot(results_right.Voltage_V, results_right.Steady_Speed_rad_per_s, 'o-', 'LineWidth', 1.5, 'DisplayName', 'Right Motor');
title('\bfMotor Speed vs Voltage');
xlabel('Voltage (V)');
ylabel('Steady Speed (rad/s)');
legend;
grid on;
hold off;

% Plot Motor Speed vs PWM (Left and Right)
figure;
plot(results_left.PWM_Command, results_left.Steady_Speed_rad_per_s, 'o-', 'LineWidth', 1.5, 'DisplayName', 'Left Motor');
hold on;
plot(results_right.PWM_Command, results_right.Steady_Speed_rad_per_s, 'o-', 'LineWidth', 1.5, 'DisplayName', 'Right Motor');
title('\bfMotor Speed vs PWM');
xlabel('PWM Command');
ylabel('Steady Speed (rad/s)');
legend;
grid on;
hold off;

%% Example Speed Plot
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
