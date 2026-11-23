clear; clc; close all;


J  = 0.05;          
Tdist = 0.0015;  
sigma_w = deg2rad(0.2);
tau_max = 0.005;
Kp = 0.05;         
Kd = 0.10;          
Ki = 0.009;         

A = [0, 1; 0, 0];
B = [0; 1/J];
Q = diag([1, 0.1]);
R = 10000;
K_lqr = lqr(A, B, Q, R);


dt = 0.01;         
t  = 0:dt:15;       
N  = length(t);


theta_ref = deg2rad(45);  
int_error = 0;
theta     = zeros(1, N);  
omega     = zeros(1, N);   
tau       = zeros(1, N);   


for k = 1:N-1
    
    error = theta_ref - theta(k);
    int_error = int_error + error * dt;
    int_error = max(min(int_error, 0.3), -0.3);
    noise = sigma_w * randn;

    tau_cmd = - Kd*(omega(k) + noise) + Kp*error + Ki*int_error;

    if tau_cmd > tau_max
        tau(k) = tau_max;
    elseif tau_cmd < -tau_max
        tau(k) = -tau_max;
    else
        tau(k) = tau_cmd;
    end
    tau_output = tau(k) + Tdist;
    alpha = tau_output / J;
    
    omega(k+1) = omega(k) + alpha*dt;
    theta(k+1) = theta(k) + omega(k)*dt;

end
error = theta_ref - theta(k);
int_error = int_error + error * dt;
int_error = max(min(int_error, 0.3), -0.3);
noise = sigma_w * randn;
tau(N) = Kp*error - Kd*(omega(N) + noise) + Ki*int_error;




figure;
subplot(2,1,1);
plot(t, rad2deg(theta), 'LineWidth', 2); hold on;
yline(45, '--r', 'Target (45 deg)');
ylabel('Attitude Angle (deg)'); grid on;
title('1-Axis CubeSat Attitude: Saturated PID, Disturbance, Anti-Windup & Gyro Noise');

subplot(2,1,2);
plot(t, tau * 1000, 'LineWidth', 2);
ylabel('Wheel Torque (mN\cdotm)'); xlabel('Time (s)'); grid on;

exportgraphics(gcf, 'cubesat_results.png', 'Resolution', 300);
