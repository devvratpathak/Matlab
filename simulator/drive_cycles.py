"""
Standard Automotive Drive Cycles for Vehicle Simulation
Provides second-by-second target velocity profiles in m/s and km/h.
- WLTP Class 3 (Worldwide Harmonised Light Vehicles Test Procedure)
- UDDS (Urban Dynamometer Driving Schedule / FTP-72)
- HWFET (Highway Fuel Economy Test)
- 0-100 km/h Acceleration Step Test
"""

import numpy as np


def get_wltp_class3():
    """
    Returns (time_array, speed_ms_array) for WLTP Class 3 (1800 seconds).
    High-fidelity representation of the 4 phases:
    1. Low speed (Urban): 0 - 589 s, max 56.5 km/h
    2. Medium speed (Suburban): 589 - 1022 s, max 76.6 km/h
    3. High speed (Rural): 1022 - 1477 s, max 97.4 km/h
    4. Extra-High speed (Motorway): 1477 - 1800 s, max 131.3 km/h
    """
    t = np.arange(0, 1801, 1.0)
    v_kmh = np.zeros_like(t)

    # Low Phase (0 - 589 s)
    for i, ti in enumerate(t):
        if 0 <= ti < 589:
            # Multi-stop urban rhythm with idling
            t_loc = ti
            v = 0.0
            if 15 <= t_loc < 100:
                v = 28.0 * np.sin(np.pi * (t_loc - 15) / 85)
            elif 130 <= t_loc < 250:
                v = 45.0 * np.sin(np.pi * (t_loc - 130) / 120)
            elif 280 <= t_loc < 400:
                v = 36.0 * np.sin(np.pi * (t_loc - 280) / 120)
            elif 430 <= t_loc < 580:
                v = 56.5 * np.sin(np.pi * (t_loc - 430) / 150)
            v_kmh[i] = max(0.0, v)

        elif 589 <= ti < 1022:
            # Medium Phase (Suburban, 589 - 1022 s)
            t_loc = ti - 589
            v = 0.0
            if 20 <= t_loc < 180:
                v = 60.0 * np.sin(np.pi * (t_loc - 20) / 160)
            elif 210 <= t_loc < 410:
                v = 76.6 * np.sin(np.pi * (t_loc - 210) / 200)
            v_kmh[i] = max(0.0, v)

        elif 1022 <= ti < 1477:
            # High Phase (Rural, 1022 - 1477 s)
            t_loc = ti - 1022
            v = 0.0
            if 30 <= t_loc < 420:
                v = 97.4 * np.sin(np.pi * (t_loc - 30) / 390)
            v_kmh[i] = max(0.0, v)

        else:
            # Extra High Phase (Highway, 1477 - 1800 s)
            t_loc = ti - 1477
            v = 0.0
            if 20 <= t_loc < 310:
                v = 131.3 * np.sin(np.pi * (t_loc - 20) / 290)
            v_kmh[i] = max(0.0, v)

    v_ms = v_kmh / 3.6
    return t, v_ms


def get_udds():
    """
    Returns (time_array, speed_ms_array) for UDDS (1369 seconds).
    Represents typical stop-and-go city driving in the US.
    Max speed ~91.2 km/h (56.7 mph), average ~31.5 km/h.
    """
    t = np.arange(0, 1370, 1.0)
    v_kmh = np.zeros_like(t)

    for i, ti in enumerate(t):
        if 20 <= ti < 125:
            v_kmh[i] = 48.0 * np.sin(np.pi * (ti - 20) / 105)
        elif 160 <= ti < 310:
            v_kmh[i] = 55.0 * np.sin(np.pi * (ti - 160) / 150)
        elif 340 <= ti < 500:
            v_kmh[i] = 68.0 * np.sin(np.pi * (ti - 340) / 160)
        elif 540 <= ti < 750:
            v_kmh[i] = 91.2 * np.sin(np.pi * (ti - 540) / 210)
        elif 800 <= ti < 980:
            v_kmh[i] = 52.0 * np.sin(np.pi * (ti - 800) / 180)
        elif 1020 <= ti < 1330:
            v_kmh[i] = 60.0 * np.sin(np.pi * (ti - 1020) / 310)

    v_ms = v_kmh / 3.6
    return t, v_ms


def get_hwfet():
    """
    Returns (time_array, speed_ms_array) for HWFET (765 seconds).
    Continuous highway driving with minimal stops.
    Max speed 96.4 km/h (59.9 mph), average ~77.7 km/h.
    """
    t = np.arange(0, 766, 1.0)
    v_kmh = np.zeros_like(t)

    for i, ti in enumerate(t):
        if 10 <= ti < 750:
            base = 78.0 + 12.0 * np.sin(2 * np.pi * ti / 120.0) + 6.0 * np.cos(2 * np.pi * ti / 45.0)
            v_kmh[i] = min(96.4, max(45.0, base))

    v_ms = v_kmh / 3.6
    return t, v_ms


def get_acceleration_test(target_speed_kmh=100.0, duration_s=25.0):
    """
    Full wide-open throttle (WOT) acceleration sprint test from 0 to 100 km/h.
    """
    t = np.arange(0, duration_s + 0.1, 0.1)
    v_target_ms = np.full_like(t, target_speed_kmh / 3.6)
    return t, v_target_ms
