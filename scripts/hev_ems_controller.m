function [mode, clutch_state, T_eng_cmd, T_mot_cmd, T_brake_fric] = hev_ems_controller(T_req, v_veh, soc, gear_ratio, final_drive, hev)
%% HEV P2 Supervisory Energy Management Strategy (EMS) Controller
% =========================================================================
% Decides operating modes and splits torque between ICE and P2 Electric Motor.
%
% Inputs:
%   T_req        - Driver requested torque at the transmission input (Nm)
%   v_veh        - Vehicle forward velocity (m/s)
%   soc          - Current battery state of charge [0.0 to 1.0]
%   gear_ratio   - Current transmission gear ratio
%   final_drive  - Final drive differential ratio
%   hev          - Vehicle parameter structure (loaded from hev_p2_params.m)
%
% Outputs:
%   mode         - Operating mode identifier:
%                  1 = PURE_EV (clutch open, engine off, motor drives)
%                  2 = ENGINE_ONLY (clutch closed, engine drives, motor idle)
%                  3 = HYBRID_BOOST (clutch closed, engine + motor deliver torque)
%                  4 = CHARGING_LOAD_SHIFT (engine over-torques, motor generates)
%                  5 = REGEN_BRAKING (clutch open, motor absorbs kinetic energy)
%   clutch_state - 0 = Open (disengaged), 1 = Closed (engaged)
%   T_eng_cmd    - Commanded engine torque (Nm)
%   T_mot_cmd    - Commanded electric motor torque (Nm, positive=motoring, negative=generating)
%   T_brake_fric - Friction braking torque on wheels/shaft (Nm)
% =========================================================================

v_kmh = v_veh * 3.6;

% Transmission shaft angular speed
total_ratio = gear_ratio * final_drive;
w_wheel = v_veh / hev.veh.wheel_radius;
w_trans_in = w_wheel * total_ratio;
w_trans_rpm = w_trans_in * (60 / (2 * pi));

%% 1. Deceleration & Braking Case (T_req < 0)
if T_req < 0
    mode = 5; % REGEN_BRAKING
    clutch_state = 0; % Disengage ICE to prevent engine pumping losses
    T_eng_cmd = 0;
    
    % Maximum available motor regen torque at current speed
    T_mot_max_regen = -min(hev.mot.max_torque, (hev.mot.max_power_kW * 1000) / max(w_trans_in, 10.0));
    
    % Battery overcharge protection
    if soc >= hev.ems.soc_high
        % Taper off regen if battery is nearly full
        regen_scale = max(0.0, (1.0 - soc) / (1.0 - hev.ems.soc_high));
        T_mot_max_regen = T_mot_max_regen * regen_scale;
    end
    
    % Allocate braking torque: motor regen first, rest to friction brakes
    T_mot_demand = T_req * hev.ems.regen_max_torque_ratio;
    T_mot_cmd = max(T_mot_demand, T_mot_max_regen);
    
    % Remaining braking torque handled by friction disc brakes
    T_brake_fric = T_req - T_mot_cmd;
    return;
end

%% 2. Zero Acceleration / Idle Case
if T_req == 0
    if soc < hev.ems.soc_low && v_veh < 1.0
        % Stationary battery recharging (ICE idles and spins motor as generator)
        mode = 4;
        clutch_state = 1;
        T_eng_cmd = 40.0; % Light engine torque
        T_mot_cmd = -40.0;% Motor operates as generator
        T_brake_fric = 0;
    else
        mode = 1;
        clutch_state = 0;
        T_eng_cmd = 0;
        T_mot_cmd = 0;
        T_brake_fric = 0;
    end
    return;
end

%% 3. Traction / Driving Case (T_req > 0)
T_brake_fric = 0;

% Check feasibility of Pure Electric Mode (EV)
can_ev = (soc > hev.ems.soc_low) && ...
         (v_kmh <= hev.ems.ev_speed_max_kmh) && ...
         (T_req <= min(hev.ems.ev_torque_max_Nm, hev.mot.max_torque));

if can_ev
    % MODE 1: PURE ELECTRIC DRIVE
    mode = 1;
    clutch_state = 0;
    T_eng_cmd = 0;
    T_mot_cmd = T_req;
    return;
end

% Engine must be running for speeds above EV threshold or when SOC is depleted
clutch_state = 1;

% Determine optimal engine torque for current speed (BSFC sweet spot)
% Typically 120-180 Nm in mid-RPMs for a 1.5L turbo
T_eng_sweet = 140.0; 

% Max available engine torque at current engine RPM
if w_trans_rpm < hev.ice.idle_rpm
    T_eng_max = interp1(hev.ice.speed_rpm_vec, hev.ice.max_torque_vec, hev.ice.idle_rpm, 'linear', 'extrap');
else
    T_eng_max = interp1(hev.ice.speed_rpm_vec, hev.ice.max_torque_vec, min(w_trans_rpm, hev.ice.max_rpm), 'linear', 'extrap');
end

% Case A: Battery low -> Engine Load Point Shifting (Charging)
if soc < hev.ems.soc_target && (T_req < T_eng_sweet)
    mode = 4; % CHARGING_LOAD_SHIFT
    % Push engine into high-efficiency zone, use surplus torque to charge battery
    T_eng_cmd = min(T_eng_sweet, T_eng_max);
    T_mot_cmd = T_req - T_eng_cmd; % Negative -> acts as generator
    return;
end

% Case B: High torque demand exceeds sweet spot or engine maximum -> Hybrid Boost
if T_req > T_eng_max
    mode = 3; % HYBRID_BOOST
    T_eng_cmd = T_eng_max;
    T_deficit = T_req - T_eng_max;
    
    if soc > hev.ems.soc_low
        T_mot_cmd = min(T_deficit, hev.mot.max_torque);
    else
        % Battery depleted, motor cannot assist fully
        T_mot_cmd = 0;
    end
    return;
end

% Case C: Standard Engine-Dominant Cruising
if (T_req <= T_eng_max) && (T_req >= T_eng_sweet - 30)
    mode = 2; % ENGINE_ONLY
    T_eng_cmd = T_req;
    T_mot_cmd = 0;
    return;
end

% Case D: Moderate demand with healthy battery -> Motor Assist or EV
if soc >= hev.ems.soc_target
    mode = 1;
    clutch_state = 0;
    T_eng_cmd = 0;
    T_mot_cmd = min(T_req, hev.mot.max_torque);
else
    mode = 2;
    clutch_state = 1;
    T_eng_cmd = T_req;
    T_mot_cmd = 0;
end

end
