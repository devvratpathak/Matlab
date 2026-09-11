"""
High-Fidelity Longitudinal Physics & Powertrain Dynamics for HEV P2
Matches the mathematical formulation of MathWorks Powertrain Blockset.
"""

import numpy as np
from scipy.interpolate import RegularGridInterpolator, interp1d


class HEVP2Vehicle:
    def __init__(self):
        # 1. Environmental Constants
        self.g = 9.81  # m/s^2
        self.rho_air = 1.205  # kg/m^3
        self.grade = 0.0  # rad

        # 2. Vehicle Glider
        self.mass = 1480.0  # kg
        self.rot_inertia_factor = 1.05
        self.effective_mass = self.mass * self.rot_inertia_factor
        self.Cd = 0.28  # Aero drag coefficient
        self.Af = 2.22  # Frontal area (m^2)
        self.cr = 0.011  # Rolling resistance coefficient
        self.wheel_radius = 0.315  # m (215/55 R17)

        # 3. Internal Combustion Engine (1.5L Turbo SI)
        self.ice_displacement = 1.498  # L
        self.ice_idle_rpm = 800.0
        self.ice_max_rpm = 6000.0
        self.ice_speed_rpm_vec = np.array(
            [800, 1200, 1600, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000], dtype=float
        )
        self.ice_max_torque_vec = np.array(
            [180, 230, 250, 250, 250, 250, 240, 225, 210, 195, 180, 160], dtype=float
        )
        self.ice_torque_vec = np.array(
            [20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 250], dtype=float
        )

        # BSFC Map (g/kWh) - shape (13, 12)
        self.bsfc_map = np.array(
            [
                [520, 470, 430, 410, 420, 440, 470, 500, 540, 590, 640, 710],
                [410, 370, 340, 325, 330, 345, 365, 390, 420, 460, 510, 560],
                [350, 310, 285, 275, 280, 290, 305, 325, 350, 380, 420, 460],
                [310, 275, 255, 245, 250, 258, 270, 285, 305, 330, 360, 400],
                [290, 255, 240, 232, 235, 242, 252, 265, 282, 305, 330, 365],
                [280, 245, 230, 225, 226, 232, 241, 253, 268, 288, 312, 342],
                [275, 240, 226, 222, 223, 228, 236, 247, 260, 278, 300, 328],
                [270, 238, 224, 220, 221, 225, 233, 243, 255, 272, 292, 318],
                [272, 237, 223, 221, 222, 226, 234, 244, 256, 273, 294, 320],
                [278, 240, 226, 224, 225, 230, 238, 248, 260, 278, 300, 328],
                [288, 248, 232, 230, 232, 238, 247, 258, 272, 290, 314, 342],
                [300, 260, 242, 240, 242, 248, 258, 270, 285, 305, 330, 360],
                [310, 270, 250, 248, 250, 255, 265, 278, 295, 315, 342, 375],
            ],
            dtype=float,
        )

        self.fuel_density_kg_per_L = 0.745  # kg/L

        # Interpolators for Engine
        self._interp_ice_max_torque = interp1d(
            self.ice_speed_rpm_vec,
            self.ice_max_torque_vec,
            kind="linear",
            fill_value="extrapolate",
        )
        self._interp_bsfc = RegularGridInterpolator(
            (self.ice_torque_vec, self.ice_speed_rpm_vec),
            self.bsfc_map,
            bounds_error=False,
            fill_value=None,
        )

        # 4. P2 Electric Motor / Generator (PMSM 50 kW)
        self.mot_max_power_kW = 50.0
        self.mot_max_torque = 170.0  # Nm
        self.mot_base_rpm = 2800.0
        self.mot_max_rpm = 7500.0

        # 5. Li-Ion Battery Pack
        self.batt_capacity_Ah = 4.5
        self.batt_num_cells = 96
        self.batt_nom_voltage = 355.2  # V
        self.batt_soc_vec = np.linspace(0.0, 1.0, 11)
        self.batt_voc_pack = (
            np.array(
                [3.20, 3.50, 3.65, 3.72, 3.78, 3.82, 3.87, 3.93, 4.01, 4.10, 4.20]
            )
            * self.batt_num_cells
        )
        self.batt_rint_dis = (
            np.array(
                [
                    0.016,
                    0.010,
                    0.007,
                    0.006,
                    0.0055,
                    0.0055,
                    0.0055,
                    0.006,
                    0.007,
                    0.008,
                    0.011,
                ]
            )
            * self.batt_num_cells
        )
        self.batt_rint_chg = (
            np.array(
                [
                    0.015,
                    0.009,
                    0.007,
                    0.006,
                    0.0055,
                    0.0055,
                    0.0055,
                    0.006,
                    0.0065,
                    0.0075,
                    0.010,
                ]
            )
            * self.batt_num_cells
        )

        self._interp_voc = interp1d(
            self.batt_soc_vec, self.batt_voc_pack, kind="linear", fill_value="extrapolate"
        )
        self._interp_rint_dis = interp1d(
            self.batt_soc_vec, self.batt_rint_dis, kind="linear", fill_value="extrapolate"
        )
        self._interp_rint_chg = interp1d(
            self.batt_soc_vec, self.batt_rint_chg, kind="linear", fill_value="extrapolate"
        )

        # 6. Driveline & Transmission (6-Speed DCT)
        self.gear_ratios = [3.82, 2.20, 1.45, 1.05, 0.82, 0.68]
        self.final_drive = 3.94
        self.driveline_eff = 0.96

    def select_gear(self, v_veh_ms):
        """Standard automatic transmission shift schedule based on vehicle speed."""
        v_kmh = v_veh_ms * 3.6
        if v_kmh < 18.0:
            return 1, self.gear_ratios[0]
        elif v_kmh < 35.0:
            return 2, self.gear_ratios[1]
        elif v_kmh < 55.0:
            return 3, self.gear_ratios[2]
        elif v_kmh < 75.0:
            return 4, self.gear_ratios[3]
        elif v_kmh < 100.0:
            return 5, self.gear_ratios[4]
        else:
            return 6, self.gear_ratios[5]

    def get_max_engine_torque(self, engine_rpm):
        rpm_clamped = np.clip(engine_rpm, self.ice_idle_rpm, self.ice_max_rpm)
        return float(self._interp_ice_max_torque(rpm_clamped))

    def get_fuel_flow(self, engine_torque, engine_rpm, is_idling=False):
        """Calculates instantaneous fuel consumption in g/s and L/s."""
        if is_idling:
            # Idle fuel burn (~0.70 L/hour = 0.145 g/s for 1.5L engine)
            fuel_gps = 0.145
            fuel_lps = (fuel_gps / 1000.0) / self.fuel_density_kg_per_L
            return fuel_gps, fuel_lps

        if engine_torque <= 1.0 or engine_rpm < 400.0:
            return 0.0, 0.0

        p_kw = (engine_torque * engine_rpm * (2 * np.pi / 60)) / 1000.0
        t_clamped = np.clip(engine_torque, self.ice_torque_vec[0], self.ice_torque_vec[-1])
        rpm_clamped = np.clip(engine_rpm, self.ice_speed_rpm_vec[0], self.ice_speed_rpm_vec[-1])

        bsfc_pt = float(self._interp_bsfc(np.array([[t_clamped, rpm_clamped]]))[0])
        fuel_gps = (p_kw * bsfc_pt) / 3600.0  # grams per second
        fuel_lps = (fuel_gps / 1000.0) / self.fuel_density_kg_per_L
        return fuel_gps, fuel_lps


    def step_battery(self, p_elec_W, soc, dt=0.1):
        """
        Equivalent circuit model step for Li-ion battery.
        Positive p_elec_W = discharge (motoring)
        Negative p_elec_W = charge (regeneration / generator)
        """
        soc_clamped = np.clip(soc, 0.05, 0.98)
        voc = float(self._interp_voc(soc_clamped))

        if p_elec_W >= 0:
            # Discharge
            rint = float(self._interp_rint_dis(soc_clamped))
            p_term = p_elec_W / 0.94  # Inverter + motor drive efficiency
        else:
            # Charge
            rint = float(self._interp_rint_chg(soc_clamped))
            p_term = p_elec_W * 0.90  # Inverter + regen capture efficiency

        # Quadratic equation for current: P = V_term * I = (Voc - I * R) * I
        # R * I^2 - Voc * I + P = 0
        discriminant = voc**2 - 4.0 * rint * p_term
        if discriminant < 0:
            # Power limit exceeded; clamp current
            i_batt = voc / (2.0 * rint)
        else:
            i_batt = (voc - np.sqrt(discriminant)) / (2.0 * rint)

        # Coulomb counting integration
        q_batt_as = self.batt_capacity_Ah * 3600.0
        delta_soc = -(i_batt * dt) / q_batt_as
        soc_next = np.clip(soc + delta_soc, 0.05, 0.98)

        v_terminal = voc - i_batt * rint
        return soc_next, i_batt, v_terminal, p_term

    def compute_resistive_forces(self, v_veh_ms):
        """Computes aerodynamic, rolling, and grade resistance forces (N)."""
        f_aero = 0.5 * self.rho_air * self.Cd * self.Af * (v_veh_ms**2)
        if v_veh_ms > 0.05:
            f_roll = self.cr * self.mass * self.g * np.cos(self.grade)
        else:
            f_roll = 0.0
        f_grade = self.mass * self.g * np.sin(self.grade)
        return f_aero, f_roll, f_grade
