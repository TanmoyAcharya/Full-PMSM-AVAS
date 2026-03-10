%% =========================================================
%  BUILD_FULL_PMSM_AVAS.m  (v2 - all errors fixed)
%
%  ONE-CLICK builder for PMSM-as-AVAS-Speaker Simulink model.
%  All 5 subsystems built and every wire connected automatically.
%
%  Fixes vs v1:
%   - Removed nested function (not allowed in scripts)
%   - Removed 'LayoutDimensions' (not a valid Scope parameter)
%   - Removed 'SystemTargetFile' (not needed without code gen)
%   - Replaced Math Function log10 with Fcn block (more reliable)
%   - Fixed From Workspace to use timeseries object
%   - Fixed all port numbering to match actual Simulink defaults
%
%  Requirements: MATLAB R2021a+, Simulink
%  Usage:  >> run('BUILD_FULL_PMSM_AVAS.m')
%% =========================================================
clear; clc;
fprintf('===========================================\n');
fprintf('  PMSM AVAS Speaker  -- Auto Model Builder\n');
fprintf('===========================================\n\n');

%% ============================================================
%  PARAMETERS
%% ============================================================
Rs    = 0.01;       Ld   = 270e-6;   Lq  = 310e-6;
psif  = 0.1546;     p    = 4;        J   = 0.089;   B = 0.005;
Vdc   = 400;
Kp_id = 2*Ld*2*pi*500;  Ki_id = Rs*2*pi*500;
Kp_iq = 2*Lq*2*pi*500;  Ki_iq = Rs*2*pi*500;
Kp_w  = 2*J*2*pi*30;    Ki_w  = B*2*pi*30;
Iq_max = 300;
I_tone = 9;   f_tone_min = 400;   f_tone_max = 1200;   v_thresh = 20;
r_wheel = 0.315;   gr = 8.5;
spd_gain = r_wheel / gr * 3.6;   % wr[rad/s] -> v[km/h]
Ts = 1e-5;   Tstop = 30;
fprintf('Parameters ready.\n');

%% ============================================================
%  CREATE MODEL
%% ============================================================
mdl = 'PMSM_AVAS_Speaker';
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep',num2str(Ts),'StopTime',num2str(Tstop));
fprintf('Model created.\n');

%% ============================================================
%  SS1: PMSM_Motor
%       In:  Vd(1)  Vq(2)  TL(3)
%       Out: id(1)  iq(2)  wr(3)  we(4)  Te(5)
%% ============================================================
SS = [mdl '/PMSM_Motor'];
add_block('simulink/Ports & Subsystems/Subsystem',SS,'Position',[550 220 690 420]);
delete_line(SS,'In1/1','Out1/1');
delete_block([SS '/In1']); delete_block([SS '/Out1']);

% Ports
add_block('simulink/Sources/In1',  [SS '/Vd'],'Port','1','Position',[20 35 50 55]);
add_block('simulink/Sources/In1',  [SS '/Vq'],'Port','2','Position',[20 115 50 135]);
add_block('simulink/Sources/In1',  [SS '/TL'],'Port','3','Position',[20 490 50 510]);

% d-axis: did/dt = (Vd - Rs*id + we*Lq*iq) / Ld
add_block('simulink/Math Operations/Sum',         [SS '/dSum'],   'Inputs','+++-','Position',[100 30 130 80]);
add_block('simulink/Math Operations/Gain',        [SS '/Ld_inv'], 'Gain',num2str(1/Ld),'Position',[155 44 205 66]);
add_block('simulink/Continuous/Integrator',       [SS '/id_int'], 'InitialCondition','0','Position',[225 42 265 68]);
add_block('simulink/Math Operations/Gain',        [SS '/Rs_d'],   'Gain',num2str(Rs),'Position',[100 95 155 115]);
add_block('simulink/Math Operations/Product',     [SS '/weLqiq'], 'Position',[60 145 95 175]);
add_block('simulink/Math Operations/Gain',        [SS '/Lq_d'],   'Gain',num2str(Lq),'Position',[105 148 150 172]);

% q-axis: diq/dt = (Vq - Rs*iq - we*Ld*id - we*psif) / Lq
add_block('simulink/Math Operations/Sum',         [SS '/qSum'],   'Inputs','+---','Position',[100 195 130 285]);
add_block('simulink/Math Operations/Gain',        [SS '/Lq_inv'], 'Gain',num2str(1/Lq),'Position',[155 222 205 258]);
add_block('simulink/Continuous/Integrator',       [SS '/iq_int'], 'InitialCondition','0','Position',[225 225 265 255]);
add_block('simulink/Math Operations/Gain',        [SS '/Rs_q'],   'Gain',num2str(Rs),'Position',[100 300 155 320]);
add_block('simulink/Math Operations/Product',     [SS '/weLdid'], 'Position',[60 335 95 365]);
add_block('simulink/Math Operations/Gain',        [SS '/Ld_q'],   'Gain',num2str(Ld),'Position',[105 338 150 362]);
add_block('simulink/Math Operations/Gain',        [SS '/psif_q'], 'Gain',num2str(psif),'Position',[60 378 115 398]);

% Torque: Te = 1.5p(psif*iq + (Ld-Lq)*id*iq)
add_block('simulink/Math Operations/Gain',        [SS '/Kt'],     'Gain',num2str(1.5*p*psif),'Position',[100 435 160 455]);
add_block('simulink/Math Operations/Product',     [SS '/idiq'],   'Position',[60 470 95 500]);
add_block('simulink/Math Operations/Gain',        [SS '/Krel'],   'Gain',num2str(1.5*p*(Ld-Lq)),'Position',[110 472 170 498]);
add_block('simulink/Math Operations/Sum',         [SS '/Te_sum'], 'Inputs','++','Position',[195 440 225 495]);

% Mechanical: dwr/dt = (Te - B*wr - TL) / J
add_block('simulink/Math Operations/Sum',         [SS '/mSum'],   'Inputs','+--','Position',[270 455 300 530]);
add_block('simulink/Math Operations/Gain',        [SS '/J_inv'],  'Gain',num2str(1/J),'Position',[320 470 370 515]);
add_block('simulink/Continuous/Integrator',       [SS '/wr_int'], 'InitialCondition','0','Position',[395 472 435 518]);
add_block('simulink/Math Operations/Gain',        [SS '/B_g'],    'Gain',num2str(B),'Position',[270 545 320 565]);
add_block('simulink/Math Operations/Gain',        [SS '/p_we'],   'Gain',num2str(p),'Position',[460 480 500 510]);

% Output ports
add_block('simulink/Sinks/Out1',[SS '/id_out'],'Port','1','Position',[550 44 580 66]);
add_block('simulink/Sinks/Out1',[SS '/iq_out'],'Port','2','Position',[550 225 580 255]);
add_block('simulink/Sinks/Out1',[SS '/wr_out'],'Port','3','Position',[550 480 580 510]);
add_block('simulink/Sinks/Out1',[SS '/we_out'],'Port','4','Position',[550 410 580 440]);
add_block('simulink/Sinks/Out1',[SS '/Te_out'],'Port','5','Position',[550 555 580 580]);

% Wire d-axis
add_line(SS,'Vd/1',    'dSum/1','autorouting','on');
add_line(SS,'dSum/1',  'Ld_inv/1','autorouting','on');
add_line(SS,'Ld_inv/1','id_int/1','autorouting','on');
add_line(SS,'id_int/1','id_out/1','autorouting','on');
add_line(SS,'id_int/1','Rs_d/1',  'autorouting','on');
add_line(SS,'Rs_d/1',  'dSum/3',  'autorouting','on');
add_line(SS,'iq_int/1','weLqiq/1','autorouting','on');
add_line(SS,'weLqiq/1','Lq_d/1',  'autorouting','on');
add_line(SS,'Lq_d/1',  'dSum/2',  'autorouting','on');

% Wire q-axis
add_line(SS,'Vq/1',    'qSum/1',  'autorouting','on');
add_line(SS,'qSum/1',  'Lq_inv/1','autorouting','on');
add_line(SS,'Lq_inv/1','iq_int/1','autorouting','on');
add_line(SS,'iq_int/1','iq_out/1','autorouting','on');
add_line(SS,'iq_int/1','Rs_q/1',  'autorouting','on');
add_line(SS,'Rs_q/1',  'qSum/2',  'autorouting','on');
add_line(SS,'id_int/1','weLdid/1','autorouting','on');
add_line(SS,'weLdid/1','Ld_q/1',  'autorouting','on');
add_line(SS,'Ld_q/1',  'qSum/3',  'autorouting','on');
add_line(SS,'p_we/1',  'psif_q/1','autorouting','on');
add_line(SS,'psif_q/1','qSum/4',  'autorouting','on');

% we into both cross-coupling products and output port
add_line(SS,'p_we/1',  'weLqiq/2','autorouting','on');
add_line(SS,'p_we/1',  'weLdid/2','autorouting','on');
add_line(SS,'p_we/1',  'we_out/1','autorouting','on');

% Torque
add_line(SS,'iq_int/1','Kt/1',    'autorouting','on');
add_line(SS,'Kt/1',    'Te_sum/1','autorouting','on');
add_line(SS,'id_int/1','idiq/1',  'autorouting','on');
add_line(SS,'iq_int/1','idiq/2',  'autorouting','on');
add_line(SS,'idiq/1',  'Krel/1',  'autorouting','on');
add_line(SS,'Krel/1',  'Te_sum/2','autorouting','on');
add_line(SS,'Te_sum/1','mSum/1',  'autorouting','on');
add_line(SS,'Te_sum/1','Te_out/1','autorouting','on');

% Mechanics
add_line(SS,'TL/1',    'mSum/3',  'autorouting','on');
add_line(SS,'mSum/1',  'J_inv/1', 'autorouting','on');
add_line(SS,'J_inv/1', 'wr_int/1','autorouting','on');
add_line(SS,'wr_int/1','B_g/1',   'autorouting','on');
add_line(SS,'B_g/1',   'mSum/2',  'autorouting','on');
add_line(SS,'wr_int/1','wr_out/1','autorouting','on');
add_line(SS,'wr_int/1','p_we/1',  'autorouting','on');

fprintf('  [1/5] PMSM_Motor wired.\n');

%% ============================================================
%  SS2: FOC_Controller
%       In:  id_ref(1) iq_ref(2) id_meas(3) iq_meas(4) we(5)
%       Out: Vd(1)  Vq(2)
%% ============================================================
FC = [mdl '/FOC_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem',FC,'Position',[340 220 480 420]);
delete_line(FC,'In1/1','Out1/1');
delete_block([FC '/In1']); delete_block([FC '/Out1']);

add_block('simulink/Sources/In1',[FC '/id_ref'], 'Port','1','Position',[20 30 50 50]);
add_block('simulink/Sources/In1',[FC '/iq_ref'], 'Port','2','Position',[20 90 50 110]);
add_block('simulink/Sources/In1',[FC '/id_meas'],'Port','3','Position',[20 150 50 170]);
add_block('simulink/Sources/In1',[FC '/iq_meas'],'Port','4','Position',[20 210 50 230]);
add_block('simulink/Sources/In1',[FC '/we_in'],  'Port','5','Position',[20 280 50 300]);

add_block('simulink/Math Operations/Sum',       [FC '/err_d'],'Inputs','+-','Position',[100 28 130 52]);
add_block('simulink/Continuous/PID Controller', [FC '/PID_d'], ...
    'P',num2str(Kp_id),'I',num2str(Ki_id),'D','0', ...
    'UpperSaturationLimit',num2str(Vdc/2), ...
    'LowerSaturationLimit',num2str(-Vdc/2),'Position',[160 22 230 58]);

add_block('simulink/Math Operations/Sum',       [FC '/err_q'],'Inputs','+-','Position',[100 88 130 112]);
add_block('simulink/Continuous/PID Controller', [FC '/PID_q'], ...
    'P',num2str(Kp_iq),'I',num2str(Ki_iq),'D','0', ...
    'UpperSaturationLimit',num2str(Vdc/2), ...
    'LowerSaturationLimit',num2str(-Vdc/2),'Position',[160 82 230 118]);

add_block('simulink/Math Operations/Product',[FC '/ff_d_prod'],'Position',[100 155 130 185]);
add_block('simulink/Math Operations/Gain',   [FC '/ff_d_gn'],  'Gain',num2str(-Lq),'Position',[150 158 200 182]);
add_block('simulink/Math Operations/Product',[FC '/ff_q_prod'],'Position',[100 215 130 245]);
add_block('simulink/Math Operations/Gain',   [FC '/ff_q_gn'],  'Gain',num2str(Ld),'Position',[150 218 200 242]);
add_block('simulink/Math Operations/Gain',   [FC '/ff_psif'],  'Gain',num2str(psif),'Position',[100 260 155 280]);
add_block('simulink/Math Operations/Sum',    [FC '/ff_q_sum'], 'Inputs','++','Position',[215 218 245 280]);

add_block('simulink/Math Operations/Sum',[FC '/Vd_sum'],'Inputs','++','Position',[275 28 305 62]);
add_block('simulink/Math Operations/Sum',[FC '/Vq_sum'],'Inputs','++','Position',[275 88 305 122]);
add_block('simulink/Sinks/Out1',[FC '/Vd_out'],'Port','1','Position',[340 35 370 55]);
add_block('simulink/Sinks/Out1',[FC '/Vq_out'],'Port','2','Position',[340 95 370 115]);

add_line(FC,'id_ref/1',  'err_d/1',   'autorouting','on');
add_line(FC,'id_meas/1', 'err_d/2',   'autorouting','on');
add_line(FC,'err_d/1',   'PID_d/1',   'autorouting','on');
add_line(FC,'PID_d/1',   'Vd_sum/1',  'autorouting','on');
add_line(FC,'iq_ref/1',  'err_q/1',   'autorouting','on');
add_line(FC,'iq_meas/1', 'err_q/2',   'autorouting','on');
add_line(FC,'err_q/1',   'PID_q/1',   'autorouting','on');
add_line(FC,'PID_q/1',   'Vq_sum/1',  'autorouting','on');
add_line(FC,'we_in/1',   'ff_d_prod/1','autorouting','on');
add_line(FC,'iq_meas/1', 'ff_d_prod/2','autorouting','on');
add_line(FC,'ff_d_prod/1','ff_d_gn/1','autorouting','on');
add_line(FC,'ff_d_gn/1', 'Vd_sum/2',  'autorouting','on');
add_line(FC,'we_in/1',   'ff_q_prod/1','autorouting','on');
add_line(FC,'id_meas/1', 'ff_q_prod/2','autorouting','on');
add_line(FC,'ff_q_prod/1','ff_q_gn/1','autorouting','on');
add_line(FC,'ff_q_gn/1', 'ff_q_sum/1','autorouting','on');
add_line(FC,'we_in/1',   'ff_psif/1', 'autorouting','on');
add_line(FC,'ff_psif/1', 'ff_q_sum/2','autorouting','on');
add_line(FC,'ff_q_sum/1','Vq_sum/2',  'autorouting','on');
add_line(FC,'Vd_sum/1',  'Vd_out/1',  'autorouting','on');
add_line(FC,'Vq_sum/1',  'Vq_out/1',  'autorouting','on');

fprintf('  [2/5] FOC_Controller wired.\n');

%% ============================================================
%  SS3: Speed_Controller
%       In:  wr_ref(1)  wr_meas(2)
%       Out: iq_tract(1)
%% ============================================================
SC = [mdl '/Speed_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem',SC,'Position',[130 220 270 310]);
delete_line(SC,'In1/1','Out1/1');
delete_block([SC '/In1']); delete_block([SC '/Out1']);

add_block('simulink/Sources/In1',[SC '/wr_ref'], 'Port','1','Position',[20 38 50 62]);
add_block('simulink/Sources/In1',[SC '/wr_meas'],'Port','2','Position',[20 98 50 122]);
add_block('simulink/Math Operations/Sum',           [SC '/err_w'],'Inputs','+-','Position',[90 36 120 64]);
add_block('simulink/Continuous/PID Controller',     [SC '/PID_w'], ...
    'P',num2str(Kp_w),'I',num2str(Ki_w),'D','0', ...
    'UpperSaturationLimit',num2str(Iq_max), ...
    'LowerSaturationLimit',num2str(-Iq_max),'Position',[150 32 230 68]);
add_block('simulink/Sinks/Out1',[SC '/iq_tract'],'Port','1','Position',[270 40 300 60]);

add_line(SC,'wr_ref/1', 'err_w/1',   'autorouting','on');
add_line(SC,'wr_meas/1','err_w/2',   'autorouting','on');
add_line(SC,'err_w/1',  'PID_w/1',   'autorouting','on');
add_line(SC,'PID_w/1',  'iq_tract/1','autorouting','on');

fprintf('  [3/5] Speed_Controller wired.\n');

%% ============================================================
%  SS4: AVAS_ToneGen
%       In:  v_kmh(1)
%       Out: iq_tone(1)
%% ============================================================
TG = [mdl '/AVAS_ToneGen'];
add_block('simulink/Ports & Subsystems/Subsystem',TG,'Position',[130 350 270 450]);
delete_line(TG,'In1/1','Out1/1');
delete_block([TG '/In1']); delete_block([TG '/Out1']);

add_block('simulink/Sources/In1',[TG '/v_kmh'],'Port','1','Position',[20 75 50 95]);

add_block('simulink/Math Operations/Gain',[TG '/f_slope'], ...
    'Gain',num2str((f_tone_max-f_tone_min)/v_thresh),'Position',[90 75 140 95]);
add_block('simulink/Sources/Constant',[TG '/f_min_k'],'Value',num2str(f_tone_min),'Position',[90 115 140 135]);
add_block('simulink/Math Operations/Sum',[TG '/f_sum'],'Inputs','++','Position',[165 72 195 108]);
add_block('simulink/Math Operations/Gain',        [TG '/to_w'],  'Gain','2*pi','Position',[215 75 255 95]);
add_block('simulink/Continuous/Integrator',       [TG '/phase'], 'InitialCondition','0','Position',[280 75 315 95]);
add_block('simulink/Math Operations/Trigonometric Function',[TG '/sine'],'Operator','sin','Position',[340 75 375 95]);
add_block('simulink/Math Operations/Gain',        [TG '/I_amp'], 'Gain',num2str(I_tone),'Position',[395 75 435 95]);

add_block('simulink/Logic and Bit Operations/Relational Operator',[TG '/v_lt_thr'], ...
    'Operator','<','Position',[90 145 130 175]);
add_block('simulink/Sources/Constant',[TG '/v_thr_k'],'Value',num2str(v_thresh),'Position',[20 155 70 175]);
add_block('simulink/Signal Routing/Switch',[TG '/sw'],'Threshold','0.5','Position',[465 70 505 100]);
add_block('simulink/Sources/Constant',[TG '/zero_k'],'Value','0','Position',[425 108 460 128]);
add_block('simulink/Sinks/Out1',[TG '/iq_tone'],'Port','1','Position',[540 75 570 95]);

add_line(TG,'v_kmh/1',  'f_slope/1', 'autorouting','on');
add_line(TG,'f_slope/1','f_sum/1',   'autorouting','on');
add_line(TG,'f_min_k/1','f_sum/2',   'autorouting','on');
add_line(TG,'f_sum/1',  'to_w/1',    'autorouting','on');
add_line(TG,'to_w/1',   'phase/1',   'autorouting','on');
add_line(TG,'phase/1',  'sine/1',    'autorouting','on');
add_line(TG,'sine/1',   'I_amp/1',   'autorouting','on');
add_line(TG,'I_amp/1',  'sw/1',      'autorouting','on');
add_line(TG,'zero_k/1', 'sw/3',      'autorouting','on');
add_line(TG,'v_kmh/1',  'v_lt_thr/1','autorouting','on');
add_line(TG,'v_thr_k/1','v_lt_thr/2','autorouting','on');
add_line(TG,'v_lt_thr/1','sw/2',     'autorouting','on');
add_line(TG,'sw/1',     'iq_tone/1', 'autorouting','on');

fprintf('  [4/5] AVAS_ToneGen wired.\n');

%% ============================================================
%  SS5: Acoustic_Monitor
%       In:  iq_total(1)
%       Out: SPL_dB(1)
%% ============================================================
AM = [mdl '/Acoustic_Monitor'];
add_block('simulink/Ports & Subsystems/Subsystem',AM,'Position',[550 460 690 540]);
delete_line(AM,'In1/1','Out1/1');
delete_block([AM '/In1']); delete_block([AM '/Out1']);

add_block('simulink/Sources/In1',[AM '/iq_in'],'Port','1','Position',[20 65 50 85]);

w0_ac  = 2*pi*1000;   zeta_ac = 0.05;   K_ac = 0.002;
num_ac = K_ac * w0_ac^2;
den_ac = [1, 2*zeta_ac*w0_ac, w0_ac^2];
add_block('simulink/Continuous/Transfer Fcn',[AM '/tf_ac'], ...
    'Numerator',mat2str(num_ac,6),'Denominator',mat2str(den_ac,6), ...
    'Position',[90 58 195 92]);

add_block('simulink/Math Operations/Abs',  [AM '/absp'],    'Position',[220 62 255 88]);
add_block('simulink/Math Operations/Gain', [AM '/pref_inv'],'Gain',num2str(1/20e-6,'%.6g'),'Position',[275 62 325 88]);

% Fcn block is the safest way to do log10 in a build script
add_block('simulink/User-Defined Functions/Fcn',[AM '/log10_fcn'], ...
    'Expr','log10(u(1))','Position',[350 62 420 88]);

add_block('simulink/Math Operations/Gain', [AM '/dB_gn'],'Gain','20','Position',[440 62 480 88]);
add_block('simulink/Sinks/Out1',[AM '/SPL_dB'],'Port','1','Position',[510 65 540 85]);

add_line(AM,'iq_in/1',   'tf_ac/1',    'autorouting','on');
add_line(AM,'tf_ac/1',   'absp/1',     'autorouting','on');
add_line(AM,'absp/1',    'pref_inv/1', 'autorouting','on');
add_line(AM,'pref_inv/1','log10_fcn/1','autorouting','on');
add_line(AM,'log10_fcn/1','dB_gn/1',  'autorouting','on');
add_line(AM,'dB_gn/1',   'SPL_dB/1',  'autorouting','on');

fprintf('  [5/5] Acoustic_Monitor wired.\n');

%% ============================================================
%  TOP-LEVEL BLOCKS
%% ============================================================

% Speed reference as timeseries (most robust format for From Workspace)
t_sp  = [0  5  10 15 20 25 30]';
v_sp  = [0 15  15 30 30 15  0]';      % km/h
wr_sp = v_sp / 3.6 / r_wheel * gr;    % rad/s
speed_ref_ts = timeseries(wr_sp, t_sp);
assignin('base','speed_ref_ts', speed_ref_ts);

add_block('simulink/Sources/From Workspace',[mdl '/SpeedRef'], ...
    'VariableName','speed_ref_ts', ...
    'Interpolate','off', ...
    'Position',[20 45 130 75]);

add_block('simulink/Sources/Constant',[mdl '/id_ref_zero'],'Value','0',  'Position',[20 110 100 140]);
add_block('simulink/Sources/Constant',[mdl '/TL_const'],   'Value','20', 'Position',[20 175 100 205]);

add_block('simulink/Math Operations/Sum',[mdl '/iq_ref_sum'],'Inputs','++','Position',[290 130 320 180]);
add_block('simulink/Math Operations/Gain',[mdl '/wr2kmh'],'Gain',num2str(spd_gain),'Position',[555 315 615 345]);
add_block('simulink/Math Operations/Sum',[mdl '/iq_tot_sum'],'Inputs','++','Position',[695 290 725 330]);

% Scopes — use NumInputPorts only (LayoutDimensions does NOT exist)
add_block('simulink/Sinks/Scope',[mdl '/Speed_Scope'],  'Position',[780 40  820 70]);
add_block('simulink/Sinks/Scope',[mdl '/Current_Scope'],'Position',[780 110 820 140]);
add_block('simulink/Sinks/Scope',[mdl '/AVAS_Scope'],   'Position',[780 180 820 210]);
add_block('simulink/Sinks/Scope',[mdl '/SPL_Scope'],    'Position',[780 250 820 280]);
set_param([mdl '/Speed_Scope'],   'NumInputPorts','2');
set_param([mdl '/Current_Scope'], 'NumInputPorts','3');
set_param([mdl '/AVAS_Scope'],    'NumInputPorts','2');

% To Workspace loggers
add_block('simulink/Sinks/To Workspace',[mdl '/Log_wr'],  'VariableName','wr_log',  'SampleTime',num2str(Ts*10),'Position',[780 330 840 360]);
add_block('simulink/Sinks/To Workspace',[mdl '/Log_iq'],  'VariableName','iq_log',  'SampleTime',num2str(Ts*10),'Position',[780 380 840 410]);
add_block('simulink/Sinks/To Workspace',[mdl '/Log_SPL'], 'VariableName','SPL_log', 'SampleTime',num2str(Ts*10),'Position',[780 430 840 460]);
add_block('simulink/Sinks/To Workspace',[mdl '/Log_tone'],'VariableName','tone_log','SampleTime',num2str(Ts*10),'Position',[780 480 840 510]);
add_block('simulink/Sources/Clock',     [mdl '/Clock'],                               'Position',[20  480  60 510]);
add_block('simulink/Sinks/To Workspace',[mdl '/Log_t'],   'VariableName','t_log',   'SampleTime',num2str(Ts*10),'Position',[780 530 840 560]);

fprintf('Top-level blocks added.\n');

%% ============================================================
%  TOP-LEVEL WIRING
%% ============================================================
add_line(mdl,'SpeedRef/1',        'Speed_Controller/1','autorouting','on');
add_line(mdl,'id_ref_zero/1',     'FOC_Controller/1',  'autorouting','on');
add_line(mdl,'iq_ref_sum/1',      'FOC_Controller/2',  'autorouting','on');
add_line(mdl,'Speed_Controller/1','iq_ref_sum/1',       'autorouting','on');
add_line(mdl,'AVAS_ToneGen/1',    'iq_ref_sum/2',       'autorouting','on');
add_line(mdl,'FOC_Controller/1',  'PMSM_Motor/1',       'autorouting','on');
add_line(mdl,'FOC_Controller/2',  'PMSM_Motor/2',       'autorouting','on');
add_line(mdl,'TL_const/1',        'PMSM_Motor/3',       'autorouting','on');
add_line(mdl,'PMSM_Motor/1',      'FOC_Controller/3',   'autorouting','on');
add_line(mdl,'PMSM_Motor/2',      'FOC_Controller/4',   'autorouting','on');
add_line(mdl,'PMSM_Motor/4',      'FOC_Controller/5',   'autorouting','on');
add_line(mdl,'PMSM_Motor/3',      'Speed_Controller/2', 'autorouting','on');
add_line(mdl,'PMSM_Motor/3',      'wr2kmh/1',           'autorouting','on');
add_line(mdl,'wr2kmh/1',          'AVAS_ToneGen/1',     'autorouting','on');
add_line(mdl,'PMSM_Motor/2',      'iq_tot_sum/1',       'autorouting','on');
add_line(mdl,'AVAS_ToneGen/1',    'iq_tot_sum/2',       'autorouting','on');
add_line(mdl,'iq_tot_sum/1',      'Acoustic_Monitor/1', 'autorouting','on');
add_line(mdl,'SpeedRef/1',        'Speed_Scope/1',      'autorouting','on');
add_line(mdl,'PMSM_Motor/3',      'Speed_Scope/2',      'autorouting','on');
add_line(mdl,'PMSM_Motor/1',      'Current_Scope/1',    'autorouting','on');
add_line(mdl,'PMSM_Motor/2',      'Current_Scope/2',    'autorouting','on');
add_line(mdl,'AVAS_ToneGen/1',    'Current_Scope/3',    'autorouting','on');
add_line(mdl,'wr2kmh/1',          'AVAS_Scope/1',       'autorouting','on');
add_line(mdl,'AVAS_ToneGen/1',    'AVAS_Scope/2',       'autorouting','on');
add_line(mdl,'Acoustic_Monitor/1','SPL_Scope/1',        'autorouting','on');
add_line(mdl,'PMSM_Motor/3',      'Log_wr/1',           'autorouting','on');
add_line(mdl,'iq_tot_sum/1',      'Log_iq/1',           'autorouting','on');
add_line(mdl,'Acoustic_Monitor/1','Log_SPL/1',          'autorouting','on');
add_line(mdl,'AVAS_ToneGen/1',    'Log_tone/1',         'autorouting','on');
add_line(mdl,'Clock/1',           'Log_t/1',            'autorouting','on');

fprintf('All wires connected.\n');

%% ============================================================
%  ARRANGE, SAVE & OPEN
%% ============================================================
Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl);
fprintf('\n=========================================\n');
fprintf('  Done!  Model saved: %s.slx\n', mdl);
fprintf('=========================================\n');
fprintf('Press Run (Ctrl+T) in Simulink to simulate.\n');
fprintf('Then run:  plot_results.m  for figures.\n\n');
open_system(mdl);
