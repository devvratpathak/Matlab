"""
Supervisory Energy Management Strategy (EMS) & Driver Controller
Controls torque split between ICE and P2 PMSM Electric Motor.
"""

import numpy as np


class LongitudinalDriver:
    """Standard Feedforward + PI Longitudinal Driver Model (matches Powertrain Blockset)."""

    def __init__(self, kp=1.2, ki=0.12):
        self.kp = kp
        self.ki = ki
        self.int_error = 0.0

    def step(self, v_target_ms, v_actual_ms, total_ratio, veh_model, dt=0.1, v_target_next=None):
        err = v_target_ms - v_actual_ms
        self.int_error += err * dt
        self.int_error = np.clip(self.int_error, -15.0, 15.0)

        # Desired acceleration (feedforward + feedback)
        if v_target_next is not None:
            a_ff = (v_target_next - v_target_ms) / dt
        else:
            a_ff = 0.0

        a_dem = a_ff + self.kp * err + self.ki * self.int_error

        # Resistive forces at current speed
        f_aero, f_roll, _ = veh_model.compute_resistive_forces(v_actual_ms)

        if a_dem >= 0:
            f_dem = veh_model.effective_mass * a_dem + f_aero + f_roll
        else:
            f_dem = veh_model.effective_mass * a_dem + f_aero

        t_wheel = f_dem * veh_model.wheel_radius
        t_req = t_wheel / max(total_ratio * veh_model.driveline_eff, 0.1)
        t_req = float(np.clip(t_req, -450.0, 380.0))

        accel_cmd = max(0.0, min(1.0, t_req / 250.0)) if t_req >= 0 else 0.0
        brake_cmd = max(0.0, min(1.0, abs(t_req) / 350.0)) if t_req < 0 else 0.0

        return accel_cmd, brake_cmd, t_req



class SupervisoryEMS:
    """
    Supervisory Rule-Based Energy Management Strategy for P2 Parallel HEV.
    Optimizes fuel efficiency, keeps battery SOC centered around target setpoint,
    and maximizes regenerative braking recovery.
    """

    def __init__(self, soc_target=0.60, soc_low=0.40, soc_high=0.75):
        self.soc_target = soc_target
        self.soc_low = soc_low
        self.soc_high = soc_high
        self.ev_speed_max_kmh = 48.0  # km/h
        self.ev_torque_max_Nm = 120.0  # Nm
        self.regen_max_torque_ratio = 0.85
        self.ice_sweet_torque = 140.0  # Optimal BSFC island torque

    def decide_torque_split(self, t_req, v_veh_ms, soc, w_shaft_rads, veh_model):
        """
        Returns:
            mode: int (1=EV, 2=Engine, 3=Boost, 4=Charge, 5=Regen)
            t_eng: float (Nm)
            t_mot: float (Nm)
            t_fric: float (Nm)
            clutch: int (0=Open, 1=Closed)
        """
        v_kmh = v_veh_ms * 3.6
        w_shaft_rpm = w_shaft_rads * (60.0 / (2.0 * np.pi))

        # -------------------------------------------------------------
        # 1. Braking & Deceleration Case (T_req < 0)
        # -------------------------------------------------------------
        if t_req < 0.0:
            mode = 5  # REGEN_BRAKING
            clutch = 0  # Disconnect engine to avoid pumping drag
            t_eng = 0.0

            # Max available regenerative motor torque at current speed
            w_eff = max(w_shaft_rads, 10.0)
            t_mot_max_regen = -min(
                veh_model.mot_max_torque,
                (veh_model.mot_max_power_kW * 1000.0) / w_eff,
            )

            # Battery protection: taper off regen as SOC approaches upper limit
            if soc >= self.soc_high:
                scale = max(0.0, (1.0 - soc) / (1.0 - self.soc_high))
                t_mot_max_regen *= scale

            t_mot_demand = t_req * self.regen_max_torque_ratio
            t_mot = max(t_mot_demand, t_mot_max_regen)  # negative numbers

            # Remainder of braking torque supplied by mechanical disc friction brakes
            t_fric = t_req - t_mot
            return mode, t_eng, t_mot, t_fric, clutch

        # -------------------------------------------------------------
        # 2. Idle / Standstill Case (T_req == 0)
        # -------------------------------------------------------------
        if abs(t_req) < 1e-3:
            t_fric = 0.0
            if soc < self.soc_low and v_veh_ms < 0.5:
                # Stationary charging: engine charges battery via P2 motor
                mode = 4
                clutch = 1
                t_eng = 45.0
                t_mot = -45.0
            else:
                mode = 1
                clutch = 0
                t_eng = 0.0
                t_mot = 0.0
            return mode, t_eng, t_mot, t_fric, clutch

        # -------------------------------------------------------------
        # 3. Driving / Propulsion Case (T_req > 0)
        # -------------------------------------------------------------
        t_fric = 0.0

        # Feasibility check for Pure EV mode
        can_ev = (
            (soc > self.soc_low)
            and (v_kmh <= self.ev_speed_max_kmh)
            and (t_req <= min(self.ev_torque_max_Nm, veh_model.mot_max_torque))
        )

        if can_ev:
            mode = 1  # PURE EV
            clutch = 0
            t_eng = 0.0
            t_mot = t_req
            return mode, t_eng, t_mot, t_fric, clutch

        # Engine must run
        clutch = 1
        t_eng_max = veh_model.get_max_engine_torque(max(w_shaft_rpm, veh_model.ice_idle_rpm))

        # Case A: Low battery SOC -> Engine Load Point Shifting (Charging)
        if (soc < self.soc_target) and (t_req < self.ice_sweet_torque):
            mode = 4  # CHARGING
            t_eng = min(self.ice_sweet_torque, t_eng_max)
            t_mot = t_req - t_eng  # Negative -> generator charges battery
            return mode, t_eng, t_mot, t_fric, clutch

        # Case B: High torque demand exceeding engine capability -> Hybrid Boost
        if t_req > t_eng_max:
            mode = 3  # HYBRID_BOOST
            t_eng = t_eng_max
            t_deficit = t_req - t_eng_max
            if soc > self.soc_low:
                t_mot = min(t_deficit, veh_model.mot_max_torque)
            else:
                t_mot = 0.0
            return mode, t_eng, t_mot, t_fric, clutch

        # Case C: Engine-dominant cruising
        if (t_req <= t_eng_max) and (t_req >= self.ice_sweet_torque - 30.0):
            mode = 2  # ENGINE ONLY
            t_eng = t_req
            t_mot = 0.0
            return mode, t_eng, t_mot, t_fric, clutch

        # Case D: Moderate demand with high battery SOC
        if soc >= self.soc_target:
            mode = 1
            clutch = 0
            t_eng = 0.0
            t_mot = min(t_req, veh_model.mot_max_torque)
        else:
            mode = 2
            clutch = 1
            t_eng = t_req
            t_mot = 0.0

        return mode, t_eng, t_mot, t_fric, clutch
