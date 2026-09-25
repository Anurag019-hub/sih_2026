











clear; clc; close all;


dt = 0.01;
T_end = 20;
t = 0:dt:T_end;
N = numel(t);

STEADY_START = 8.0;
steady_idx = t >= STEADY_START;

panTarget = 20;
tiltTarget = 10;


dir_factors = 1.00:0.05:1.50;


CURRENT_LIMIT = 2.0;

fprintf('\n===============================================================\n');
fprintf(' FINE DIRECTIONAL FEEDFORWARD SWEEP\n');
fprintf('===============================================================\n');
fprintf('Simulation step       : %.3f s\n', dt);
fprintf('Scenario duration     : %.1f s\n', T_end);
fprintf('Steady-state window   : %.1f to %.1f s\n', STEADY_START, T_end);
fprintf('Sweep                 : 100%% to 150%% in 5%% steps\n');
fprintf('Current command limit : %.2f A\n', CURRENT_LIMIT);
fprintf('===============================================================\n\n');


rng(1);
ref = runCase(t, panTarget, tiltTarget, dt, STEADY_START, ...
    0, true, false, CURRENT_LIMIT);


rng(1);
results = runCase(t, panTarget, tiltTarget, dt, STEADY_START, ...
    dir_factors(1), true, true, CURRENT_LIMIT);

for s = 2:numel(dir_factors)
    rng(1);
    results(s) = runCase(t, panTarget, tiltTarget, dt, STEADY_START, ...
        dir_factors(s), true, true, CURRENT_LIMIT);
end

mean_err = [results.mean_ss];
rms_err = [results.rms_ss];
max_err = [results.max_ss];

mean_I = [results.mean_current_ss];
rms_I = [results.rms_current_ss];
max_I = [results.max_current_ss];

violations = [results.current_limit_violations];
violation_pct = [results.current_limit_violation_pct];


[best_mean_error, best_idx] = min(mean_err);
best_factor = dir_factors(best_idx);


fprintf('COLD UNCOMPENSATED REFERENCE\n');
fprintf('-----------------------------------------------------------------\n');
fprintf('Mean error       : %.4f mrad\n', ref.mean_ss);
fprintf('RMS error        : %.4f mrad\n', ref.rms_ss);
fprintf('Max error        : %.4f mrad\n', ref.max_ss);
fprintf('Mean |I|         : %.4f A\n', ref.mean_current_ss);
fprintf('RMS |I|          : %.4f A\n', ref.rms_current_ss);
fprintf('Max |I|          : %.4f A\n', ref.max_current_ss);
fprintf('Limit violations : %d\n', ref.current_limit_violations);

fprintf('\nFINE SWEEP RESULTS\n');
fprintf('----------------------------------------------------------------------------------------------\n');
fprintf('%7s %13s %13s %13s %13s %13s %13s %11s %11s\n', ...
    'FF (%)','Mean mrad','RMS mrad','Max mrad', ...
    'Mean A','RMS A','Max A','Violations','Improve %');
fprintf('----------------------------------------------------------------------------------------------\n');

for s = 1:numel(results)
    improvement = 100*(ref.mean_ss - mean_err(s))/max(ref.mean_ss,eps);

    fprintf('%7.0f %13.4f %13.4f %13.4f %13.4f %13.4f %13.4f %11d %10.2f\n', ...
        100*dir_factors(s), ...
        mean_err(s), rms_err(s), max_err(s), ...
        mean_I(s), rms_I(s), max_I(s), ...
        violations(s), improvement);
end

fprintf('----------------------------------------------------------------------------------------------\n');

fprintf('\nBEST TESTED GAIN\n');
fprintf('  Gain                 : %.0f%%\n', 100*best_factor);
fprintf('  Mean error           : %.4f mrad\n', best_mean_error);
fprintf('  RMS error            : %.4f mrad\n', rms_err(best_idx));
fprintf('  Max error            : %.4f mrad\n', max_err(best_idx));
fprintf('  Mean current         : %.4f A\n', mean_I(best_idx));
fprintf('  RMS current          : %.4f A\n', rms_I(best_idx));
fprintf('  Max current          : %.4f A\n', max_I(best_idx));
fprintf('  Limit violations     : %d (%.3f%% of steady-state samples)\n', ...
    violations(best_idx), violation_pct(best_idx));

fprintf('\nIMPROVEMENT VS COLD UNCOMPENSATED\n');
fprintf('  Mean error reduction  : %.2f %%\n', ...
    100*(ref.mean_ss-best_mean_error)/max(ref.mean_ss,eps));
fprintf('  RMS error reduction   : %.2f %%\n', ...
    100*(ref.rms_ss-rms_err(best_idx))/max(ref.rms_ss,eps));
fprintf('  Max error reduction   : %.2f %%\n', ...
    100*(ref.max_ss-max_err(best_idx))/max(ref.max_ss,eps));
fprintf('  Mean current change   : %+.2f %%\n', ...
    100*(mean_I(best_idx)-ref.mean_current_ss)/max(ref.mean_current_ss,eps));


figure('Name','Fine Sweep - Pointing Error', ...
       'Position',[100 100 1050 700]);

plot(100*dir_factors, mean_err, '-o', ...
    'LineWidth',1.6,'MarkerSize',6);
hold on;
yline(ref.mean_ss,'--','Cold uncompensated');
plot(100*best_factor,best_mean_error,'o', ...
    'MarkerSize',10,'LineWidth',2);

xlabel('Directional feedforward factor (%)');
ylabel('Steady-state mean pointing error (mrad)');
title('Fine Feedforward Sweep: Pointing Error');
grid on;


figure('Name','Fine Sweep - Current', ...
       'Position',[150 120 1050 700]);

plot(100*dir_factors, mean_I, '-o', ...
    'LineWidth',1.6,'MarkerSize',6);
hold on;
yline(ref.mean_current_ss,'--','Cold uncompensated');

xlabel('Directional feedforward factor (%)');
ylabel('Steady-state mean |I| (A)');
title('Fine Feedforward Sweep: Mean Current');
grid on;


figure('Name','Fine Sweep - Error and Current', ...
       'Position',[200 140 1100 750]);

yyaxis left
plot(100*dir_factors, mean_err, '-o', 'LineWidth',1.6);
ylabel('Mean pointing error (mrad)');

yyaxis right
plot(100*dir_factors, mean_I, '-s', 'LineWidth',1.6);
ylabel('Mean |I| (A)');

xlabel('Directional feedforward factor (%)');
title('Accuracy vs Actuator Current Tradeoff');
grid on;


figure('Name','Best Gain - Current and Error', ...
       'Position',[250 160 1100 750]);

subplot(2,1,1);
plot(t, results(best_idx).err, 'LineWidth',1.4);
hold on;
plot(t, ref.err, 'LineWidth',1.2);
xline(STEADY_START,'--','Cold steady-state');
xlabel('Time (s)');
ylabel('Pointing error (mrad)');
title(sprintf('Pointing Error: Best Tested Gain = %.0f%%',100*best_factor));
legend('Best directional FF','Cold uncompensated','Location','northeast');
grid on;

subplot(2,1,2);
plot(t, results(best_idx).current, 'LineWidth',1.4);
hold on;
plot(t, ref.current, 'LineWidth',1.2);
yline(CURRENT_LIMIT,'--','Current limit');
xline(STEADY_START,'--','Cold steady-state');
xlabel('Time (s)');
ylabel('|I| (A)');
title('Commanded Current');
legend('Best directional FF','Cold uncompensated','Location','northeast');
grid on;


FF_Percent = (100*dir_factors).';
MeanError_mrad = mean_err.';
RMSError_mrad = rms_err.';
MaxError_mrad = max_err.';
MeanCurrent_A = mean_I.';
RMSCurrent_A = rms_I.';
MaxCurrent_A = max_I.';
CurrentLimitViolations = violations.';
CurrentLimitViolationPercent = violation_pct.';
MeanErrorReductionPercent = ...
    100*(ref.mean_ss-mean_err).'/max(ref.mean_ss,eps);

fineTable = table(FF_Percent,MeanError_mrad,RMSError_mrad,MaxError_mrad, ...
    MeanCurrent_A,RMSCurrent_A,MaxCurrent_A, ...
    CurrentLimitViolations,CurrentLimitViolationPercent, ...
    MeanErrorReductionPercent);

writetable(fineTable,'gimbal_directional_ff_fine_sweep_results.csv');

fprintf('\nResults exported to:\n');
fprintf('  gimbal_directional_ff_fine_sweep_results.csv\n');
fprintf('\nFine sweep complete.\n');



function result = runCase(t,panTarget,tiltTarget,dt,steadyStart, ...
                          ff_factor,kf_on,directional_ff,current_limit)

    N = numel(t);
    steady_idx = t >= steadyStart;

    pan = initAxis();
    tilt = initAxis();

    err_mrad = zeros(1,N);
    Ipan = zeros(1,N);
    Itilt = zeros(1,N);

    temp_t = coldRamp(t);

    for k = 1:N
        temp = temp_t(k);

        [pan,Ipan(k)] = stepAxis( ...
            pan,panTarget,temp,dt,ff_factor,kf_on,directional_ff);

        [tilt,Itilt(k)] = stepAxis( ...
            tilt,tiltTarget,temp,dt,ff_factor,kf_on,directional_ff);

        err_deg = hypot( ...
            pan.true_angle-panTarget, ...
            tilt.true_angle-tiltTarget);

        err_mrad(k) = err_deg*(pi/180)*1000;
    end

    current = hypot(Ipan,Itilt);

    e_ss = err_mrad(steady_idx);
    i_ss = current(steady_idx);

    result.t = t;
    result.err = err_mrad;
    result.current = current;

    result.mean_ss = mean(e_ss);
    result.rms_ss = sqrt(mean(e_ss.^2));
    result.max_ss = max(e_ss);

    result.mean_current_ss = mean(i_ss);
    result.rms_current_ss = sqrt(mean(i_ss.^2));
    result.max_current_ss = max(i_ss);

    result.current_limit_violations = sum(i_ss > current_limit);
    result.current_limit_violation_pct = ...
        100*result.current_limit_violations/numel(i_ss);
end

function T = coldRamp(tt)
    T = 25-(45/8)*tt;
    T(tt>8) = -20;
end

function axis = initAxis()
    axis.kf_angle = 0;
    axis.kf_bias = 0;
    axis.P = [1 0;0 1];

    axis.pid_integral = 0;
    axis.pid_prev = 0;

    axis.true_angle = 0;
    axis.true_vel = 0;
end

function I_ff = oldTemperatureFF(temp_c)
    FF_A = 0.05;
    FF_B = -0.0025;
    FF_C = 0.00004;

    I_ff = FF_A + FF_B*temp_c + FF_C*temp_c^2;
    I_ff = max(0,I_ff);
end

function Kf = frictionCurrent(temp_c)
    Kf = oldTemperatureFF(temp_c)*9.0;
end

function [axis,I_cmd] = stepAxis( ...
    axis,target_deg,temp_c,dt,ff_factor,kf_on,directional_ff)

    KP = 2.5;
    KI = 0.8;
    KD = 0.05;

    PID_OUT_MAX = 1.5;
    PID_INT_MAX = 0.5;

    QA_BASE = 0.001;
    QB_BASE = 0.003;
    R_MEAS = 0.03;

    J = 0.006;
    DAMP = 0.35;
    FRIC_SCALE = 9.0;


    gyro_bias_true = 0.15*(25-temp_c)*0.03;
    gyro_reading = axis.true_vel + gyro_bias_true + (rand-0.5)*0.4;
    enc_reading = axis.true_angle + (rand-0.5)*0.03;


    rate = gyro_reading-axis.kf_bias;
    axis.kf_angle = axis.kf_angle+dt*rate;

    dT = abs(temp_c-25);

    if kf_on
        scale = 1+0.01*dT;
    else
        scale = 1;
    end

    qA = QA_BASE*scale;
    qB = QB_BASE*scale;

    P = axis.P;

    p00 = P(1,1)+dt*(dt*P(2,2)-P(1,2)-P(2,1)+qA);
    p01 = P(1,2)-dt*P(2,2);
    p10 = P(2,1)-dt*P(2,2);
    p11 = P(2,2)+qB*dt;

    P = [p00 p01;p10 p11];


    y = enc_reading-axis.kf_angle;
    S = P(1,1)+R_MEAS;

    K0 = P(1,1)/S;
    K1 = P(2,1)/S;

    axis.kf_angle = axis.kf_angle+K0*y;
    axis.kf_bias = axis.kf_bias+K1*y;

    p00t = P(1,1);
    p01t = P(1,2);

    P(1,1) = P(1,1)-K0*p00t;
    P(1,2) = P(1,2)-K0*p01t;
    P(2,1) = P(2,1)-K1*p00t;
    P(2,2) = P(2,2)-K1*p01t;

    axis.P = P;


    err = target_deg-axis.kf_angle;

    axis.pid_integral = axis.pid_integral+err*dt;
    axis.pid_integral = max(-PID_INT_MAX, ...
                            min(PID_INT_MAX,axis.pid_integral));

    deriv = (err-axis.pid_prev)/dt;
    axis.pid_prev = err;

    I_pid = KP*err+KI*axis.pid_integral+KD*deriv;
    I_pid = max(-PID_OUT_MAX,min(PID_OUT_MAX,I_pid));


    if directional_ff
        if abs(err) < 1e-6
            direction = 0;
        else
            direction = sign(err);
        end

        I_ff = ff_factor*frictionCurrent(temp_c)*0.6*direction;
    else
        I_ff = ff_factor*oldTemperatureFF(temp_c);
    end

    I_cmd = max(-2,min(2,I_pid+I_ff));


    thresh = oldTemperatureFF(temp_c)*FRIC_SCALE;

    if abs(axis.true_vel)<0.05
        if abs(I_cmd)<=thresh
            fric = -I_cmd;
        else
            fric = -sign(I_cmd)*thresh*0.7;
        end
    else
        fric = -sign(axis.true_vel)*thresh*0.6;
    end

    net = I_cmd+fric-DAMP*axis.true_vel;

    axis.true_vel = axis.true_vel+(net/J)*dt;
    axis.true_angle = axis.true_angle+axis.true_vel*dt;
end
