# Hybrid-Electric Vehicle (HEV P2) Simulation Model

[![MATLAB & Simulink Compatible](https://img.shields.io/badge/MATLAB-R2022b%20--%20R2024b-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Simulink Model](https://img.shields.io/badge/Simulink-P2%20Parallel%20HEV-blue.svg)](https://www.mathworks.com/products/simulink.html)
[![Python Standalone Simulation](https://img.shields.io/badge/Python-3.10%2B-green.svg)](https://python.org)

An advanced, competition-grade **Parallel P2 Hybrid Electric Vehicle (HEV)** simulation model and digital twin testbed developed in **MATLAB/Simulink** and **Python**.

Designed to meet the specifications of the **MathWorks Powertrain Blockset HEV P2 Reference Application**, academic research papers, and automotive powertrain design competitions.

---

## 🏎️ Powertrain Architecture: P2 Parallel Hybrid

The P2 architecture places a **Permanent Magnet Synchronous Motor (PMSM)** between an internal combustion engine (ICE) disconnect clutch and a 6-speed Dual Clutch Transmission (DCT):
- **Internal Combustion Engine**: 1.5L Turbocharged Direct-Injection SI (110 kW / 148 HP, 250 Nm, BSFC sweet spot ~220 g/kWh)
- **P2 Electric Motor / Generator**: 50 kW (67 HP), 170 Nm peak torque, 94% peak efficiency
- **Energy Storage System**: 355.2V Lithium-Ion battery pack (96 cells in series, 1.6 kWh, 4.5 Ah)
- **Transmission**: 6-Speed Dual Clutch Transmission (DCT) + 3.94 Final Drive Differential
- **Chassis & Aerodynamics**: 1480 kg curb mass, $C_d = 0.28$, $A_f = 2.22\text{ m}^2$, $c_r = 0.011$

---

## 📊 Key Competition Results

| Driving Scenario | Conventional ICE Baseline | HEV P2 Model | Fuel Savings (%) | HEV US Fuel Economy | Regenerative Energy Captured |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **WLTP Class 3** | 4.86 L/100 km | **3.77 L/100 km** | **-22.6%** | **62.5 MPG** | 6.68 MJ |
| **UDDS (City)** | 4.55 L/100 km | **2.81 L/100 km** | **-38.2%** | **83.6 MPG** | 3.84 MJ |
| **HWFET (Highway)** | 4.46 L/100 km | **3.81 L/100 km** | **-14.5%** | **61.8 MPG** | 5.00 MJ |
| **0-100 km/h Sprint** | 12.20 s | **7.70 s** | **4.50 s faster** | Instant Boost | Combined 160 kW peak |

---

## 🚀 Quickstart Guide

### Option 1: Run in MATLAB / MATLAB Online (One-Click)
1. Open [MATLAB Online](https://matlab.mathworks.com/) or your local MATLAB installation.
2. In the MATLAB command window, run:
```matlab
run('scripts/run_simulation_and_report.m')
```
This automatically loads all parameters, generates/links the Simulink model, executes the WLTP simulation, and displays the performance dashboard figure.

### Option 2: Run Local Python Digital Twin (Instant)
```bash
python simulator/run_benchmarks.py
```
This executes WLTP, UDDS, HWFET, and the 0-100 km/h acceleration sprint, generating the plots in the `results/` folder.

---

## 📁 Repository Structure

```
Matlab/
├── params/
│   └── hev_p2_params.m            # Comprehensive vehicle & component parameters
├── scripts/
│   ├── build_hev_simulink_model.m # Programmatic Simulink model builder
│   ├── hev_ems_controller.m       # Supervisory Energy Management Strategy (VCU)
│   └── run_simulation_and_report.m# Automated test harness & report generator
├── simulator/
│   ├── drive_cycles.py            # WLTP Class 3, UDDS, HWFET velocity profiles
│   ├── hev_dynamics.py            # Longitudinal vehicle dynamics & component physics
│   ├── hev_controller.py          # Driver model & supervisory EMS state machine
│   └── run_benchmarks.py          # Full simulation runner & comparative graphs
├── docs/
│   ├── MATLAB_SIMULINK_GUIDE.md   # Beginner's step-by-step guide to MATLAB & Simulink
│   └── HEV_TECHNICAL_REPORT.md    # Formal engineering competition submission report
└── results/
    ├── hev_wltp_dashboard.png     # WLTP speed, torque split, SOC, and fuel burn
    ├── hev_acceleration_sprint.png# 0-100 km/h acceleration comparison
    └── fuel_economy_benchmark_comparison.png # Comparative bar charts
```

---

## 📖 Documentation
- **[Beginner's MATLAB & Simulink Guide](docs/MATLAB_SIMULINK_GUIDE.md)**: How to get MATLAB Online for free with your competition voucher, load the files, and present your project to judges.
- **[Full Technical Report](docs/HEV_TECHNICAL_REPORT.md)**: Mathematical equations, component modeling, and control theory details.
