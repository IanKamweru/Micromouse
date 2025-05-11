%% Left Motor
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
