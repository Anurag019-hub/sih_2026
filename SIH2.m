clear; clc; close all;

dt = 0.1;                       
t_final = 400;                  
time = 0:dt:t_final;
N = length(time);

T_amb = -30.0;                  
RH = 85.0;                      

C_rf     = 12.0;                
C_motor  = 90.0;                
C_sink   = 110.0;               
C_batt   = 180.0;               

R_TIM_rf    = 0.15;             
R_TIM_motor = 0.30;             
R_sink_min  = 0.20;             
R_sink_max  = 2.80;             
R_batt_amb  = 3.00;             

C_lens   = 8.0;                 
C_radome = 45.0;                
R_lens_amb   = 2.2;             
R_radome_amb = 1.8;             

P_ptc_max    = 40.0;            
P_ito_max    = 12.0;            
P_radome_max = 25.0;            

T_rf_target = 45.0;             
Kp_fan = 6.0; Ki_fan = 0.10; Kd_fan = 2.0; 
integral_err_fan = 0; prev_err_fan = 0;

T_lens_ramp_setpoint = T_amb;   
max_lens_ramp_rate = 3.0 / 60;  

T_rf     = zeros(1,N); T_rf(1)     = T_amb;
T_motor  = zeros(1,N); T_motor(1)  = T_amb;
T_sink   = zeros(1,N); T_sink(1)   = T_amb;
T_batt   = zeros(1,N); T_batt(1)   = T_amb;
T_lens   = zeros(1,N); T_lens(1)   = T_amb;
T_radome = zeros(1,N); T_radome(1) = T_amb;

PWM_fan       = zeros(1,N); 
PWM_ito_lens  = zeros(1,N);
P_ptc_cmd     = zeros(1,N);
P_radome_cmd  = zeros(1,N);
Main_Power_En = zeros(1,N);

P_rf_input    = zeros(1,N);     
P_motor_input = zeros(1,N);     

for k = 1:N
    if (time(k) >= 130 && time(k) <= 200)
        P_rf_input(k) = 60.0;
    elseif (time(k) >= 280 && time(k) <= 340)
        P_rf_input(k) = 80.0;
    end
    
    if time(k) >= 160 && time(k) <= 240
        P_motor_input(k) = 35.0;
    end
end

is_preheated = false;

for k = 1:N-1
    if T_batt(k) < -10.0 && ~is_preheated
        P_ptc_cmd(k) = P_ptc_max;       
        Main_Power_En(k) = 0;           
    else
        is_preheated = true;
        P_ptc_cmd(k) = 0.0;             
        Main_Power_En(k) = 1;           
    end
    
    if Main_Power_En(k) == 0
        PWM_fan(k) = 0.0;               
    else
        if (P_rf_input(k) >= 50.0) || (P_motor_input(k) >= 30.0)
            PWM_fan(k) = 1.0;           
            integral_err_fan = 0;       
        else
            err_fan = T_rf(k) - T_rf_target;
            if abs(err_fan) < 10.0
                integral_err_fan = integral_err_fan + err_fan * dt;
            else
                integral_err_fan = 0;
            end
            deriv_fan = (err_fan - prev_err_fan) / dt;
            prev_err_fan = err_fan;
            
            u_pid = (Kp_fan * err_fan) + (Ki_fan * integral_err_fan) + (Kd_fan * deriv_fan);
            pwm_calc = 0.15 + (u_pid / 100.0); 
            PWM_fan(k) = min(max(pwm_calc, 0.15), 1.0);
        end
    end
    
    a = 17.27; b = 237.7;
    alpha = ((a * T_amb) / (b + T_amb)) + log(RH / 100.0);
    T_dew = (b * alpha) / (a - alpha);
    
    frost_risk = (T_amb <= 2.0) && (T_lens(k) <= (T_dew + 2.0));
    
    if frost_risk
        T_lens_target_final = max(3.0, T_dew + 3.0);
        
        if T_lens_ramp_setpoint < T_lens_target_final
            T_lens_ramp_setpoint = min(T_lens_target_final, T_lens_ramp_setpoint + max_lens_ramp_rate * dt);
        end
        
        err_lens = T_lens_ramp_setpoint - T_lens(k);
        PWM_ito_lens(k) = min(1.0, max(0.0, err_lens * 0.40));
        
        P_radome_cmd(k) = P_radome_max; 
    else
        T_lens_ramp_setpoint = T_lens(k);
        PWM_ito_lens(k) = 0.0;
        P_radome_cmd(k)  = 0.0;
    end
    
    R_sink_eff = R_sink_max - PWM_fan(k) * (R_sink_max - R_sink_min);
    
    dT_batt = (P_ptc_cmd(k) - (T_batt(k) - T_amb) / R_batt_amb) / C_batt;
    
    q_rf_to_sink = (T_rf(k) - T_sink(k)) / R_TIM_rf;
    dT_rf = (P_rf_input(k) - q_rf_to_sink) / C_rf;
    
    q_motor_to_sink = (T_motor(k) - T_sink(k)) / R_TIM_motor;
    dT_motor = (P_motor_input(k) - q_motor_to_sink) / C_motor;
    
    q_sink_to_amb = (T_sink(k) - T_amb) / R_sink_eff;
    dT_sink = ((q_rf_to_sink + q_motor_to_sink) - q_sink_to_amb) / C_sink;
    
    P_ito_actual = PWM_ito_lens(k) * P_ito_max;
    dT_lens = (P_ito_actual - (T_lens(k) - T_amb) / R_lens_amb) / C_lens;
    
    dT_radome = (P_radome_cmd(k) - (T_radome(k) - T_amb) / R_radome_amb) / C_radome;
    
    T_batt(k+1)   = T_batt(k)   + dT_batt   * dt;
    T_rf(k+1)     = T_rf(k)     + dT_rf     * dt;
    T_motor(k+1)  = T_motor(k)  + dT_motor  * dt;
    T_sink(k+1)   = T_sink(k)   + dT_sink   * dt;
    T_lens(k+1)   = T_lens(k)   + dT_lens   * dt;
    T_radome(k+1) = T_radome(k) + dT_radome * dt;
end

PWM_fan(N)      = PWM_fan(N-1);
PWM_ito_lens(N) = PWM_ito_lens(N-1);
P_ptc_cmd(N)    = P_ptc_cmd(N-1);
P_radome_cmd(N) = P_radome_cmd(N-1);
Main_Power_En(N)= Main_Power_En(N-1);

figure('Name', 'SIH26050 Unified TMS Simulation');

subplot(4,1,1);
plot(time, T_rf, 'r-', 'LineWidth', 2); hold on;
plot(time, T_motor, 'c-', 'LineWidth', 1.8);
plot(time, T_sink, 'm--', 'LineWidth', 1.5);
yline(T_rf_target, 'r:', 'RF Temp Target (45°C)');
grid on; ylabel('Cooling Zone (°C)');
title('Cooling System: High-Power Component Response (RF Amp & Tracking Motor)');
legend('RF Junction Temp', 'Motor Temp', 'Heat Sink Temp', 'Location', 'northeast');

subplot(4,1,2);
plot(time, T_batt, 'b-', 'LineWidth', 2); hold on;
plot(time, T_lens, 'g-', 'LineWidth', 2);
plot(time, T_radome, 'y--', 'LineWidth', 1.5);
yline(-10, 'b:', 'Batt Cold Limit (-10°C)');
yline(0, 'r:', 'Freezing Threshold (0°C)');
grid on; ylabel('Heating Zone (°C)');
title('Heating & De-Icing System: Battery Cold Start & Optics Frost Protection');
legend('Battery Pack Temp', 'Camera Lens Glass Temp', 'Radome Housing Temp', 'Location', 'southeast');

subplot(4,1,3);
plot(time, P_rf_input, 'r-', 'LineWidth', 1.5); hold on;
plot(time, P_motor_input, 'c-', 'LineWidth', 1.5);
grid on; ylabel('Heat Inputs (W)');
title('Disturbance Profile: RF Jamming Pulses & Gimbal Motor Heat Generation');
legend('RF Burst Dissipation (W)', 'Motor Dissipation (W)', 'Location', 'northeast');

subplot(4,1,4);
plot(time, PWM_fan * 100, 'r-', 'LineWidth', 1.8); hold on;
plot(time, PWM_ito_lens * 100, 'g-', 'LineWidth', 1.8);
plot(time, Main_Power_En * 100, 'b--', 'LineWidth', 1.5);
grid on; xlabel('Time (seconds)'); ylabel('Actuator Control (%)');
title('Actuator Outputs: Fan PWM, ITO Heater Duty Cycle, & System Interlock');
legend('Cooling Fan PWM (%)', 'Lens ITO Heater PWM (%)', 'Main System Power Enable (%)', 'Location', 'east');