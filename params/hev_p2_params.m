%% HEV P2 Simulation Model Parameters
% =========================================================================
% Hybrid Electric Vehicle (P2 Configuration) Parameter Initialization
% Architecture: ICE -> Disconnect Clutch -> P2 Electric Motor -> 6-Speed DCT -> Wheels
% Reference: MathWorks Powertrain Blockset HEV P2 Reference Application
% =========================================================================

clear hev; % Clear previous structure if exists
hev = struct();

%% 1. Global & Environmental Constants
hev.env.g           = 9.81;       % Acceleration due to gravity (m/s^2)
hev.env.rho_air     = 1.205;      % Air density at 20 deg C (kg/m^3)
hev.env.T_amb       = 293.15;     % Ambient temperature (K)
hev.env.grade       = 0.0;        % Road grade angle (rad)

%% 2. Vehicle Glider & Chassis Specifications
hev.veh.mass        = 1480;       % Base curb mass + driver (kg)
hev.veh.rot_inertia_factor = 1.05;% Factor for rotational inertias (drivetrain + wheels)
hev.veh.effective_mass = hev.veh.mass * hev.veh.rot_inertia_factor;
hev.veh.Cd          = 0.28;       % Aerodynamic drag coefficient
hev.veh.Af          = 2.22;       % Frontal cross-sectional area (m^2)
hev.veh.cr          = 0.011;      % Rolling resistance coefficient
hev.veh.wheel_radius = 0.315;     % Dynamic tire rolling radius (m) [e.g. 215/55 R17]

%% 3. Internal Combustion Engine (ICE) Specifications
% 1.5-Liter Turbocharged Spark-Ignition (SI) Direct Injection
hev.ice.displacement = 1.498;     % Displacement (Liters)
hev.ice.cylinders    = 4;
hev.ice.max_power_kW = 110;       % Rated peak power: 110 kW (148 HP) @ 5500 RPM
hev.ice.idle_rpm     = 800;       % Idle speed (RPM)
hev.ice.max_rpm      = 6000;      % Redline speed (RPM)
hev.ice.inertia      = 0.15;      % Flywheel & crankshaft inertia (kg*m^2)

% Engine Speed Vector for Maps (RPM)
hev.ice.speed_rpm_vec = [800, 1200, 1600, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000];
hev.ice.speed_rad_vec = hev.ice.speed_rpm_vec * (2 * pi / 60);

% Maximum Torque Curve (Nm) across speed vector (Wide flat plateau typical of turbo engines)
hev.ice.max_torque_vec = [180, 230, 250, 250, 250, 250, 240, 225, 210, 195, 180, 160];

% Torque Vector for 2D Maps (Nm)
hev.ice.torque_vec = [20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 250];

% Brake Specific Fuel Consumption (BSFC) Table in g/kWh
% Dimensions: (length(torque_vec) x length(speed_rpm_vec))
% Sweet spot is ~225 g/kWh around 2000-3000 RPM and 140-200 Nm
hev.ice.bsfc_map = [
    520, 470, 430, 410, 420, 440, 470, 500, 540, 590, 640, 710; % 20 Nm
    410, 370, 340, 325, 330, 345, 365, 390, 420, 460, 510, 560; % 40 Nm
    350, 310, 285, 275, 280, 290, 305, 325, 350, 380, 420, 460; % 60 Nm
    310, 275, 255, 245, 250, 258, 270, 285, 305, 330, 360, 400; % 80 Nm
    290, 255, 240, 232, 235, 242, 252, 265, 282, 305, 330, 365; % 100 Nm
    280, 245, 230, 225, 226, 232, 241, 253, 268, 288, 312, 342; % 120 Nm
    275, 240, 226, 222, 223, 228, 236, 247, 260, 278, 300, 328; % 140 Nm
    270, 238, 224, 220, 221, 225, 233, 243, 255, 272, 292, 318; % 160 Nm
    272, 237, 223, 221, 222, 226, 234, 244, 256, 273, 294, 320; % 180 Nm
    278, 240, 226, 224, 225, 230, 238, 248, 260, 278, 300, 328; % 200 Nm
    288, 248, 232, 230, 232, 238, 247, 258, 272, 290, 314, 342; % 220 Nm
    300, 260, 242, 240, 242, 248, 258, 270, 285, 305, 330, 360; % 240 Nm
    310, 270, 250, 248, 250, 255, 265, 278, 295, 315, 342, 375  % 250 Nm
];

% Fuel Properties (Gasoline / Petrol)
hev.ice.fuel_density_kg_per_L = 0.745;    % Density in kg/Liter
hev.ice.fuel_lhv_MJ_per_kg    = 44.0;     % Lower Heating Value (MJ/kg)

%% 4. P2 Electric Motor / Generator (PMSM)
% Permanent Magnet Synchronous Motor located after disconnect clutch
hev.mot.max_power_kW = 50.0;     % Peak power: 50 kW (67 HP)
hev.mot.max_torque   = 170.0;    % Peak torque: 170 Nm
hev.mot.base_rpm     = 2800;     % Base speed where field weakening starts
hev.mot.max_rpm      = 7500;     % Maximum rotational speed (RPM)
hev.mot.inertia      = 0.04;     % Rotor inertia (kg*m^2)

% Motor Speed Vector (RPM)
hev.mot.speed_rpm_vec = [0, 500, 1000, 1800, 2800, 3500, 4500, 5500, 6500, 7500];
hev.mot.speed_rad_vec = hev.mot.speed_rpm_vec * (2 * pi / 60);

% Motor Torque Vector (Nm)
hev.mot.torque_vec = [0, 20, 40, 60, 80, 100, 120, 140, 160, 170];

% Motor Efficiency Map (Dimensions: length(torque_vec) x length(speed_rpm_vec))
% Peaks at 94% efficiency in mid-torque/mid-speed zone
hev.mot.eff_map = [
    0.70, 0.75, 0.80, 0.82, 0.83, 0.83, 0.82, 0.80, 0.78, 0.75; % 0 Nm (no load)
    0.78, 0.84, 0.88, 0.90, 0.91, 0.90, 0.89, 0.87, 0.85, 0.82; % 20 Nm
    0.82, 0.88, 0.91, 0.92, 0.93, 0.92, 0.91, 0.89, 0.87, 0.84; % 40 Nm
    0.84, 0.90, 0.93, 0.94, 0.94, 0.93, 0.92, 0.90, 0.88, 0.85; % 60 Nm
    0.85, 0.91, 0.93, 0.94, 0.94, 0.93, 0.92, 0.90, 0.88, 0.85; % 80 Nm
    0.85, 0.91, 0.93, 0.94, 0.94, 0.93, 0.92, 0.90, 0.88, 0.84; % 100 Nm
    0.84, 0.90, 0.92, 0.93, 0.93, 0.92, 0.91, 0.89, 0.86, 0.82; % 120 Nm
    0.83, 0.89, 0.91, 0.92, 0.92, 0.91, 0.89, 0.87, 0.84, 0.80; % 140 Nm
    0.81, 0.87, 0.89, 0.90, 0.90, 0.89, 0.87, 0.85, 0.81, 0.77; % 160 Nm
    0.80, 0.86, 0.88, 0.89, 0.89, 0.88, 0.86, 0.83, 0.79, 0.75  % 170 Nm
];

%% 5. High-Voltage Battery Pack (Lithium-Ion)
% Sized for full hybrid electric vehicle (HEV)
hev.batt.capacity_Ah      = 4.5;        % Nominal cell capacity (Ah)
hev.batt.num_cells_series = 96;         % 96 cells in series (Nominal 355.2 V)
hev.batt.nom_voltage      = 96 * 3.7;   % 355.2 V nominal
hev.batt.total_energy_kWh = (hev.batt.nom_voltage * hev.batt.capacity_Ah) / 1000; % ~1.6 kWh
hev.batt.max_charge_kW    = 45.0;       % Maximum charge power (regen limit)
hev.batt.max_discharge_kW = 55.0;       % Maximum discharge power

% SOC Vector for Open Circuit Voltage and Internal Resistance Lookups
hev.batt.soc_vec = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];

% Open-Circuit Voltage per Cell (Volts)
hev.batt.voc_cell_vec = [3.20, 3.50, 3.65, 3.72, 3.78, 3.82, 3.87, 3.93, 4.01, 4.10, 4.20];
hev.batt.voc_pack_vec = hev.batt.voc_cell_vec * hev.batt.num_cells_series;

% Internal Resistance per Cell (Ohms)
hev.batt.rint_chg_cell = [0.015, 0.009, 0.007, 0.006, 0.0055, 0.0055, 0.0055, 0.006, 0.0065, 0.0075, 0.010];
hev.batt.rint_dis_cell = [0.016, 0.010, 0.007, 0.006, 0.0055, 0.0055, 0.0055, 0.006, 0.0070, 0.0080, 0.011];
hev.batt.rint_dis_pack = hev.batt.rint_dis_cell * hev.batt.num_cells_series;
hev.batt.rint_chg_pack = hev.batt.rint_chg_cell * hev.batt.num_cells_series;

%% 6. Transmission & Driveline (6-Speed Dual Clutch Transmission)
hev.driveline.gear_ratios  = [3.82, 2.20, 1.45, 1.05, 0.82, 0.68]; % 1st to 6th gear
hev.driveline.final_drive  = 3.94;                                  % Differential ratio
hev.driveline.efficiency   = 0.96;                                  % Driveline mechanical efficiency
hev.driveline.shift_delay  = 0.25;                                  % Shift transition time (s)

%% 7. Energy Management System (EMS) / VCU Control Thresholds
hev.ems.soc_target   = 0.60;   % Target SOC setpoint (60%)
hev.ems.soc_low      = 0.40;   % Minimum allowed SOC threshold (forced charge)
hev.ems.soc_high     = 0.75;   % High SOC threshold (favor EV / discharge)
hev.ems.ev_speed_max_kmh = 50.0; % Max vehicle speed for pure EV drive in town (km/h)
hev.ems.ev_torque_max_Nm = 120.0;% Max driver requested torque for EV mode
hev.ems.regen_max_torque_ratio = 0.85; % Max proportion of braking force handled by regen

disp('HEV P2 simulation parameters loaded successfully.');
