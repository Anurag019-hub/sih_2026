function verify_gimbal_simulink_135ff_v4





if ~evalin('base','exist(''simulink_v4_true_error_mrad'',''var'')
), error('Run the V4 Simulink model first.'); end


e=evalin('base','simulink_v4_true_error_mrad');


i=evalin('base','simulink_v4_total_current_A');


t=e.Time(:); ev=squeeze(e.Data); iv=squeeze(i.Data);


idx=(t>=8)&(t<=20);


ev=ev(idx); iv=iv(idx);


meanE=mean(ev); rmsE=sqrt(mean(ev.^2)); maxE=max(abs(ev));


meanI=mean(iv); rmsI=sqrt(mean(iv.^2)); maxI=max(abs(iv));


refE=0.3093; refI=0.9061;


fprintf('\n===============================================================\n');


fprintf(' V4 MATLAB-REFERENCE-EQUIVALENCE CHECK\n');


fprintf('===============================================================\n');


fprintf('Simulink mean steady error : %.6f mrad\n',meanE);


fprintf('Simulink RMS steady error  : %.6f mrad\n',rmsE);


fprintf('Simulink max steady error  : %.6f mrad\n',maxE);


fprintf('Simulink mean |I|          : %.6f A\n',meanI);


fprintf('Simulink RMS |I|           : %.6f A\n',rmsI);


fprintf('Simulink max |I|           : %.6f A\n',maxI);


fprintf('\nMATLAB 135%% reference mean error : %.6f mrad\n',refE);


fprintf('Absolute mean-error difference   : %.6f mrad\n',abs(meanE-refE));


fprintf('Relative mean-error difference   : %.4f %%\n',100*abs(meanE-refE)/refE);


fprintf('MATLAB 135%% reference mean |I|   : %.6f A\n',refI);


fprintf('Absolute mean-current difference  : %.6f A\n',abs(meanI-refI));


fprintf('Relative mean-current difference  : %.4f %%\n',100*abs(meanI-refI)/refI);


fprintf('===============================================================\n');


if abs(meanE-refE)<1e-6


    fprintf('PASS: mean pointing error matches the reference to numerical precision.\n');


else


    fprintf('NOTE: inspect the metrics above; the V4 model should closely reproduce the reference.\n');


end


end
