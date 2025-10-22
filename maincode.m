% clc;
% clear;
% close all;


s = tf('s');
numerator = [1 5];
denominator = [1 10 35 50 24];  
G = tf(numerator, denominator);
T_delay = 2;                   
G.InputDelay = T_delay;


pade_order = 4;
[num_pade, den_pade] = pade(T_delay, pade_order);
Delay_approx = tf(num_pade, den_pade);
G_rational = G * Delay_approx;  


[~, den_rational] = tfdata(G_rational, 'v');
omega = logspace(-1, 2, 500);
jw = 1i * omega;
P_jw = polyval(den_rational, jw);

figure;
plot(real(P_jw), imag(P_jw), 'b-', 'LineWidth', 2);
xlabel('Re[P(j\omega)]');
ylabel('Im[P(j\omega)]');
title('Mihailov Plot');
grid on;


reduced_order = 2;  % Desired reduced order
G_balred = balred(G_rational, reduced_order);
[num_red, den_red] = tfdata(G_balred, 'v');


t_vec = linspace(0, 30, 1500);
[y_full, ~] = impulse(G_rational, t_vec);
[y_red, ~] = impulse(tf(num_red, den_red), t_vec);

if abs(y_red(1)) > 1e-6
    scaling_factor = y_full(1) / y_red(1);
else
    scaling_factor = 1;
end

num_red_matched = num_red * scaling_factor;


if all(abs(num_red_matched) < 1e-10)
    warning('Numerator coefficients near zero after scaling, reverting.');
    num_red_matched = num_red;
end

G_moment_matched = tf(num_red_matched, den_red);


[y_target, ~] = step(G_rational, t_vec);

cost_fun = @(num_coeffs) sum((step(tf(num_coeffs, den_red), t_vec) - y_target).^2);

opts = optimset('Display','off','MaxIter',1000,'TolFun',1e-8,'TolX',1e-8);


[num_opt, ~] = fminsearch(cost_fun, num_red_matched, opts);

G_ede = tf(num_opt, den_red);


G_final_reduced = G_ede;
disp('Final Reduced Model:');
G_final_reduced


G_red_no_delay = G_final_reduced;  % Approx reduced model without delay (already approximated)

try
    Q = inv(G_red_no_delay);  % Invert model for IMC controller
catch
    warning('Model not invertible, setting Q to unity transfer function.');
    Q = tf(1,1);
end

lambda = 0.5; % Filter tuning parameter
F = tf(1, [lambda 1]);
Gc = minreal(Q * F);


if order(Gc) > order(G_red_no_delay)
    warning('IMC controller improper, increasing lambda to fix.');
    lambda = 1.5;
    F = tf(1, [lambda 1]);
    Gc = minreal(Q * F);
end

disp('IMC Controller:');
Gc


T_imc = feedback(G_final_reduced * Gc, 1);

figure;
step(T_imc, 30);
% title('Closed-Loop Step Response (Reduced Model)');
% xlabel('Time (s)');
% ylabel('Output');
grid on;

%% Step 10: Closed-loop System with Original System
T_orig = feedback(G * Gc, 1);
t_compare = linspace(0, 30, 1000);

[y_orig, t_orig] = step(T_orig, t_compare);
[y_red, t_red] = step(T_imc, t_compare);

figure;
plot(t_orig, y_orig, 'r-', 'LineWidth', 2); hold on;
plot(t_red, y_red, 'b--', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Output');
% title('Closed-loop Step Response Comparison: Original vs Reduced');
legend('Original', 'Reduced');
grid on;


S_orig = stepinfo(y_orig, t_orig);
S_red = stepinfo(y_red, t_red);

disp('Original System + IMC Controller Performance:');
disp(S_orig);
disp('Reduced Model Closed Loop Performance:');
disp(S_red);


figure;
step(G_rational, G_moment_matched, G_final_reduced);
legend('Original (Padé approx)', 'Reduced (Moment Matching)', 'Reduced (EDE Optimized)', 'Location', 'Best');
title('Step Response Comparison');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;


figure;
impulse(G_rational, 'b-', t_vec); hold on;
impulse(G_moment_matched, 'r--', t_vec);
impulse(G_final_reduced, 'g-.', t_vec);
legend('Original (Padé approx)', 'Reduced (Moment Matching)', 'Reduced (EDE Optimized)', 'Location', 'Best');
title('Impulse Response Comparison');
grid on;
hold off;

model = 'models';  
load_system(model);

% Step 2: Set simulation time
set_param(model, 'StopTime', '100');

% Step 3: Run the simulation
out = sim(model);

% Step 4: Extract performance index values from 'out'
IAE  = out.IAE;
ITSE = out.ITSE;
ITAE = out.ITAE;
ISE=out.ISE;

% Step 5: Plot each metric separately
figure;
plot(IAE.time, IAE.signals.values, 'b', 'LineWidth', 2);
title('IAE vs Time');
xlabel('Time (s)');
ylabel('IAE');
grid on;
% Step 5: Plot each metric separately
figure;
plot(ISE.time, ISE.signals.values, 'b', 'LineWidth', 2);
title('ISE vs Time');
xlabel('Time (s)');
ylabel('ISE');
grid on;

figure;
plot(ITSE.time, ITSE.signals.values, 'r', 'LineWidth', 2);
title('ITSE vs Time');
xlabel('Time (s)');
ylabel('ITSE');
grid on;

figure;
plot(ITAE.time, ITAE.signals.values, 'g', 'LineWidth', 2);
title('ITAE vs Time');
xlabel('Time (s)');
ylabel('ITAE');
grid on;

function J = ise_step_cost(num, den, y_ref, t)
    try
        G = tf(num, den);
        y = step(G, t);
        J = trapz(t, (y - y_ref).^2);  % ISE calculation
    catch
        J = Inf;  % If transfer function fails (e.g., unstable), return large error
    end
end


