%% Right Motor PID Controller Design (Continuous Time)

w_ref = 30;  % Reference speed in rad/s

% Motor parameters
Km_right = 2.7262;          % Motor gain
tau_motor_right = 0.055;   % Motor time constant
% Continuous-time motor transfer function for the right motor
G_right = tf(Km_right, [tau_motor_right 1]);

% PID gains (tuned for the right motor)
Kp_right = 0.17174;
Ki_right = 5.05008;
Kd_right = 0.00042;

% PID controller with derivative filter for the right motor
N = 140;  % Filter coefficient
Gc_right = tf([Kd_right Kp_right Ki_right], [1 0]) * tf([1], [1/N 1]);

% Closed-loop Transfer Functions for the right motor
Gcl_right = feedback(Gc_right * G_right, 1);  % Closed-loop: output (speed)
Gcl_u_right = feedback(Gc_right, G_right);    % Closed-loop: control effort

% Plotting
figure('Name', 'Closed-Loop PID Control for Right Motor');
set(gcf, 'Position', [100 300 1300 400]);

% Root Locus
subplot(1,3,1);
rl = rlocusplot(Gc_right * G_right);
title('\bfCompensated Root Locus (Right Motor)'); hold on;
[wn_right, zeta_right, poles_right] = damp(Gcl_right);
plot(real(poles_right), imag(poles_right), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'Color', [0.8 0 0]);
grid on;

% Response characteristics
disp('Response Characteristics for Right Motor');
fprintf('PID Gains:\n');
fprintf('  Kp = %.4f\n', Kp_right);
fprintf('  Ki = %.4f\n', Ki_right);
fprintf('  Kd = %.4f\n', Kd_right);
stepinfo(w_ref * Gcl_right)
disp('Closed-Loop Poles (Right Motor)');
damp(Gcl_right);
[y_u_right, t_u_right] = step(w_ref * Gcl_u_right);
u_max_right = max(y_u_right);
fprintf('\nMax Control Effort (Right Motor): %.3f V\n', u_max_right);

% Step Response: Speed (Right Motor)
subplot(1,3,2);
sp1 = stepplot(w_ref *Gcl_right);
sp1.Responses.LineWidth = 1.6;
sp1.Responses.Color = [0 0 0.8];
title('\bfStep Response (Right Motor Speed)');
ylabel('Speed (rad/s)');
xlabel('Time (s)');
grid on;

% Step Response: Control Effort (Right Motor)
subplot(1,3,3);
sp2 = stepplot(w_ref * Gcl_u_right);
sp2.Responses.LineWidth = 1.6;
sp2.Responses.Color = [0.8 0 0];
title('\bfControl Effort (Right Motor)');
ylabel('Voltage (V)');
xlabel('Time (s)');
grid on;

%%
% Initial PID gains (starting point)
Kp_base = 0.14312;
Ki_base = 6.012;
Kd_base = 0.00085;
N_base = 100;

% Sweeping ranges
Kp_range = 0.8 * Kp_base : 0.02 * Kp_base : 1.2 * Kp_base;
Ki_range = 0.8 * Ki_base : 0.02 * Ki_base : 1.2 * Ki_base;
Kd_range = 0.5 * Kd_base : 0.02 * Kd_base : 2 * Kd_base;
N_range = 50 : 10 : 150;

% Best result tracking
best_settling_time = inf;
best_damping = 0;
best_gains = [];

% Sweeping loop
iteration = 0;
for Kp = Kp_range
    for Ki = Ki_range
        for Kd = Kd_range
            for N = N_range
                iteration = iteration + 1;
                
                % Create PID controller with derivative filter
                Gc_right = tf([Kd Kp Ki], [1 0]) * tf([1], [1/N 1]);
                
                % Closed-loop Transfer Functions
                Gcl_right = feedback(Gc_right * G_right, 1);  % Speed
                Gcl_u_right = feedback(Gc_right, G_right);    % Control effort
                
                % Get step response data
                [y_u_right, ~] = step(w_ref * Gcl_u_right);
                u_max_right = max(y_u_right);
                
                % Check control effort limit
                if u_max_right <= 12
                    % Calculate step info
                    S = stepinfo(w_ref * Gcl_right);
                    
                    % Check if this is the best candidate so far
                    if S.SettlingTime < best_settling_time && S.Overshoot < 10
                        best_settling_time = S.SettlingTime;
                        best_damping = min(damp(Gcl_right));
                        best_gains = [Kp, Ki, Kd, N];
                        
                        % Print progress message
                        fprintf('New best found at iteration %d:\n', iteration);
                        fprintf('  Kp = %.5f, Ki = %.5f, Kd = %.5f, N = %d\n', Kp, Ki, Kd, N);
                        fprintf('  Settling Time = %.4f s, Damping = %.4f\n', best_settling_time, best_damping);
                        fprintf('  Max Control Effort = %.3f V\n', u_max_right);
                    end
                end
            end
        end
    end
end

% Display the best results
if ~isempty(best_gains)
    fprintf('\nBest PID Gains Found:\n');
    fprintf('  Kp = %.5f\n', best_gains(1));
    fprintf('  Ki = %.5f\n', best_gains(2));
    fprintf('  Kd = %.5f\n', best_gains(3));
    fprintf('  N = %d\n', best_gains(4));
    fprintf('  Settling Time = %.4f s\n', best_settling_time);
    fprintf('  Damping Ratio = %.4f\n', best_damping);
else
    fprintf('No suitable gains found within the given limits.\n');
end

%% Right Motor PID Controller Design (Discrete Time)

Ts = 0.01;  % Discrete sampling time

% Motor parameters
Km_right = 2.7262;
tau_motor_right = 0.055;

% Continuous-time motor transfer function
G_right = tf(Km_right, [tau_motor_right 1]);

% Continuous-time PID gains (from your tuned values)
Kp_right = 0.17174;
Ki_right = 5.05008;
Kd_right = 0.00042;
N = 140;

% Continuous-time PID with derivative filter
Gc_right = tf([Kd_right, Kp_right, Ki_right], [1 0]) * tf([1], [1/N 1]);

% Convert to discrete using Tustin
Gz_right = c2d(G_right, Ts, 'zoh');      % Plant discretization
Gcz_right = c2d(Gc_right, Ts, 'tustin'); % PID discretization

disp('Discrete-time Plant G(z):'); Gz_right
disp('Discrete-time Compensator Gc(z):'); Gcz_right

% Continuous-time closed loop
CLTF_r_to_c = feedback(Gc_right * G_right, 1);  % Speed
CLTF_r_to_u = feedback(Gc_right, G_right);      % Control effort

% Closed-loop Transfer Functions (Discrete)
CLTFz_r_to_c = feedback(Gcz_right * Gz_right, 1);  % Speed
CLTFz_r_to_u = feedback(Gcz_right, Gz_right);      % Control effort

% Performance Check
fprintf('\nDiscrete System Characteristics:\n');
[~, d] = damp(CLTFz_r_to_c);
s = stepinfo(w_ref * CLTFz_r_to_c);
u = stepinfo(w_ref * CLTFz_r_to_u);
fprintf('  Damping = %.2f\n', min(d));
fprintf('  Settling Time = %.3f s\n', s.SettlingTime);
fprintf('  Overshoot = %.2f %%\n', s.Overshoot);
fprintf('  Max Control Effort = %.3f V\n', u.Peak);

% Plot Results
figure('Name', 'Discrete-Time PID Control for Right Motor');
set(gcf, 'Position', [1000 300 1200 800]);
tl = tiledlayout(2,3);

% Root Locus (S-Plane)
nexttile(1);
rlocus(Gc_right * G_right);
hold on;
[~,~,p_c] = damp(CLTF_r_to_c);
plot(real(p_c), imag(p_c), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'Color', [0.8 0 0]);
title('Continuous S-Plane Root Locus');
grid on;

% Root Locus (Z-Plane)
nexttile(4);
rlocus(Gcz_right * Gz_right);
hold on;
[~,~,p_z] = damp(CLTFz_r_to_c);
plot(real(p_z), imag(p_z), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'Color', [0.8 0 0]);
title('Discrete Z-Plane Root Locus');
grid on;
xlim([-1.2 1.2]);
ylim([-1.2 1.2]);

% Continuous Output Step
nexttile(2);
step(w_ref * feedback(Gc_right * G_right, 1));
title('Continuous Output Step Response');
grid on;

% Continuous Control Effort
nexttile(3);
step(w_ref * feedback(Gc_right, G_right));
title('Continuous Control Effort Step Response');
grid on;

% Discrete Output Step
nexttile(5);
step(w_ref * CLTFz_r_to_c);
title('Discrete Output Step Response');
grid on;

% Discrete Control Effort
nexttile(6);
step(w_ref * CLTFz_r_to_u);
title('Discrete Control Effort Step Response');
grid on;
