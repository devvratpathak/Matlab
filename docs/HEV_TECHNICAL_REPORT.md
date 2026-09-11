# Hybrid Electric Vehicle (P2 Architecture) Simulation & Performance Report

**Competition Design Project: Advanced Powertrain Simulation**  
**Author / Team:** Devvr & Antigravity Engineering  
**Platform:** MATLAB / Simulink & Python Digital Twin  
**Date:** September 2026  

---

## 1. Executive Summary

This report presents the engineering design, mathematical formulation, supervisory control strategy, and simulation validation of a **Parallel P2 Hybrid Electric Vehicle (HEV)**. The model was developed to evaluate fuel economy, powertrain efficiency, battery state-of-charge (SOC) preservation, and dynamic acceleration performance under standardized international automotive test procedures (WLTP Class 3, UDDS, and HWFET).

### Key Performance Highlights:
- **Urban Fuel Economy (UDDS)**: Achieves **2.81 L/100 km (83.6 MPG)**, delivering a **38.2% reduction in fuel consumption** compared to the conventional internal combustion engine (ICE) baseline (4.55 L/100 km).
- **Combined Cycle (WLTP Class 3)**: Delivers **3.77 L/100 km (62.5 MPG)** with a **22.6% fuel savings** and **6.68 MJ of kinetic energy recovered** via regenerative braking.
- **Dynamic Acceleration (0-100 km/h)**: The combined 160 kW powertrain (110 kW ICE + 50 kW PMSM electric boost) cuts the 0-100 km/h sprint time from **12.20 seconds (ICE alone) to 7.70 seconds (HEV)**—an acceleration improvement of **4.50 seconds**.

---

## 2. Vehicle Powertrain Architecture (P2 Parallel Configuration)

The vehicle utilizes a **P2 Parallel Hybrid** topology, positioned between the internal combustion engine and a 6-speed Dual Clutch Transmission (DCT).

```
   ┌──────────────────────┐
   │  1.5L Turbocharged   │
   │    SI Engine (ICE)   │
   └──────────┬───────────┘
              │ 
              ▼
   ┌──────────────────────┐
   │ P2 Disconnect Clutch │ ◄── [Clutch Actuator Command]
   └──────────┬───────────┘
              │
              ├──────────────────────────────────┐
              ▼                                  ▼
   ┌──────────────────────┐            ┌───────────────────┐
   │  50 kW PMSM Motor /  │            │  High-Voltage     │
   │  Generator Unit      │ ◄────────► │  Li-Ion Battery   │
   └──────────┬───────────┘            │  (355V, 1.5 kWh)  │
              │                        └───────────────────┘
              ▼
   ┌──────────────────────┐
   │ 6-Speed Dual-Clutch  │
   │  Transmission (DCT)  │
   └──────────┬───────────┘
              │
              ▼
   ┌──────────────────────┐
   │ Differential & Drive │
   │ Wheels (Longitudinal)│
   └──────────────────────┘
```

### Architectural Advantages:
1. **Engine Decoupling during Pure EV Mode**: The disconnect clutch allows the electric motor to power the vehicle independently during low-speed urban driving with the engine switched completely off, avoiding crankshaft friction and pumping losses.
2. **Decoupled Regenerative Braking**: During deceleration, the clutch opens and the P2 motor captures maximum vehicle kinetic energy without spinning the engine.
3. **Torque-Summing Boost**: Under full throttle, the engine and motor torques sum algebraically at the transmission input ($T_{prop} = T_{ice} + T_{mot}$).

---

## 3. Powertrain Component Specifications

| Subsystem | Parameter | Value | Units |
| :--- | :--- | :--- | :--- |
| **Vehicle Glider** | Curb Mass + Driver ($m$) | 1480 | kg |
| | Rotational Inertia Factor ($\delta_{rot}$) | 1.05 | - |
| | Effective Mass ($m_{eff}$) | 1554 | kg |
| | Aerodynamic Drag Coeff. ($C_d$) | 0.28 | - |
| | Frontal Area ($A_f$) | 2.22 | $\text{m}^2$ |
| | Rolling Resistance ($c_r$) | 0.011 | - |
| | Tire Rolling Radius ($r_w$) | 0.315 | m |
| **Internal Combustion Engine** | Type | 1.5L Inline-4 Turbocharged Direct-Injection SI | - |
| | Peak Power | 110 (148 HP) @ 5500 RPM | kW |
| | Peak Torque | 250 @ 1600–3000 RPM | Nm |
| | Idle / Max RPM | 800 / 6000 | RPM |
| | Minimum BSFC Sweet Spot | 220–225 | g/kWh |
| **P2 Electric Motor** | Type | Permanent Magnet Synchronous Motor (PMSM) | - |
| | Peak Power | 50.0 (67 HP) | kW |
| | Peak Torque | 170.0 | Nm |
| | Base / Max RPM | 2800 / 7500 | RPM |
| | Peak Efficiency | 94% | - |
| **High-Voltage Battery** | Chemistry | Lithium-Ion (NMC) | - |
| | Cells in Series | 96 (24 modules of 4 cells) | - |
| | Nominal Voltage | 355.2 | V |
| | Capacity | 4.5 | Ah |
| | Total Stored Energy | 1.60 | kWh |
| | Max Charge / Discharge Power | 45.0 / 55.0 | kW |
| **Transmission** | Type | 6-Speed Dual Clutch Transmission (DCT) | - |
| | Gear Ratios | [3.82, 2.20, 1.45, 1.05, 0.82, 0.68] | - |
| | Final Drive Differential | 3.94 | - |
| | Mechanical Efficiency | 96% | - |

---

## 4. Mathematical Modeling & Dynamics

### 4.1. Longitudinal Vehicle Dynamics
The net acceleration of the vehicle along its longitudinal axis is governed by Newton's second law:

$$m_{eff} \cdot \frac{dv}{dt} = F_{tractive} + F_{fric\_brake} - F_{aero} - F_{roll} - F_{grade}$$

Where:
- Aerodynamic drag: $F_{aero} = \frac{1}{2} \rho_{air} C_d A_f v^2$
- Tire rolling resistance: $F_{roll} = c_r \cdot m \cdot g \cdot \cos(\theta)$
- Road grade resistance: $F_{grade} = m \cdot g \cdot \sin(\theta)$
- Tractive force at wheels: $F_{tractive} = \frac{T_{prop}}{r_w} = \frac{(T_{eng} + T_{mot}) \cdot GR \cdot FD \cdot \eta_{dl}}{r_w}$

### 4.2. Fuel Consumption Formulation
Instantaneous fuel mass flow rate is interpolated dynamically from the 2D Brake Specific Fuel Consumption (BSFC) surface map as a function of engine torque $T_{eng}$ and rotational speed $\omega_{eng}$:

$$\dot{m}_{fuel} = \frac{P_{ice}(kW) \cdot BSFC(g/kWh)}{3600} \quad [\text{g/s}]$$

$$\dot{V}_{fuel} = \frac{\dot{m}_{fuel}}{1000 \cdot \rho_{fuel}} \quad [\text{L/s}]$$

### 4.3. Battery Equivalent Circuit Model (ECM)
The terminal battery voltage is modeled using an open-circuit voltage $V_{oc}$ in series with internal resistance $R_{int}$:

$$V_{term} = V_{oc}(SOC) - I_{batt} \cdot R_{int}(SOC)$$

The battery current $I_{batt}$ required to deliver terminal electrical power $P_{term}$ is solved via quadratic formulation:

$$I_{batt} = \frac{V_{oc} - \sqrt{V_{oc}^2 - 4 R_{int} P_{term}}}{2 R_{int}}$$

State of Charge (SOC) tracking is computed via Coulomb counting:

$$SOC(t) = SOC(t_0) - \int_{t_0}^t \frac{I_{batt}(\tau)}{Q_{batt}} d\tau$$

---

## 5. Supervisory Energy Management Strategy (EMS)

The Hybrid Vehicle Control Unit (VCU) uses a finite-state machine with charge-sustaining thresholds:

| Mode | Condition | Clutch | Engine State | Motor State |
| :--- | :--- | :---: | :--- | :--- |
| **1. Pure EV** | $v < 48\text{ km/h}$, $SOC > 40\%$, $T_{req} < 120\text{ Nm}$ | Open | OFF ($T_{eng}=0$) | Driving alone ($T_{mot} = T_{req}$) |
| **2. Engine Cruise** | $v \ge 48\text{ km/h}$, moderate torque | Closed | Fired ($T_{eng}=T_{req}$) | Idle ($T_{mot}=0$) |
| **3. Hybrid Boost** | $T_{req} > T_{eng\_max}$ (Wide open throttle) | Closed | Full torque ($T_{eng}=T_{eng\_max}$) | Electric assist ($T_{mot} = T_{req} - T_{eng}$) |
| **4. Load Shifting (Charge)** | $SOC < 60\%$, light road load | Closed | High efficiency ($T_{eng}=140\text{ Nm}$) | Generator ($T_{mot} < 0$) charges battery |
| **5. Regen Braking** | $T_{req} < 0$ (Driver brakes) | Open | Fuel Cutoff ($T_{eng}=0$) | Generator ($T_{mot} < 0$) recovers kinetic energy |

---

## 6. Simulation Results & Benchmark Comparison

### 6.1. Drive Cycle Fuel Economy Results

| Test Procedure | Cycle Distance | Conventional ICE Baseline | HEV P2 Parallel Model | Fuel Reduction (%) | HEV US Fuel Economy | Recovered Regen Energy |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **WLTP Class 3** | 23.26 km | 4.86 L/100 km | **3.77 L/100 km** | **-22.6%** | **62.5 MPG** | 6.68 MJ |
| **UDDS (Urban)** | 11.99 km | 4.55 L/100 km | **2.81 L/100 km** | **-38.2%** | **83.6 MPG** | 3.84 MJ |
| **HWFET (Highway)** | 16.51 km | 4.46 L/100 km | **3.81 L/100 km** | **-14.5%** | **61.8 MPG** | 5.00 MJ |

### 6.2. Acceleration Performance
- **0 to 100 km/h Sprint (ICE Alone)**: 12.20 s
- **0 to 100 km/h Sprint (HEV P2 Combined)**: **7.70 s**
- **Advantage**: **4.50 seconds quicker** due to instant electric motor torque available from 0 RPM.

---

## 7. How to Execute the Code

### In MATLAB / Simulink:
```matlab
% In MATLAB Command Window:
cd 'C:\Users\devvr\Documents\Github\Matlab'
run('scripts/run_simulation_and_report.m')
```

### In Standalone Python Simulator:
```bash
python simulator/run_benchmarks.py
```
All generated charts are automatically exported to the `results/` directory.
