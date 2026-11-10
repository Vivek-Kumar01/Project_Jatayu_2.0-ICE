"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
PHASE 4: CRUISE-CLIMB INTEGRATION (ENERGY-HEIGHT METHOD)
================================================================================
Simultaneous climb and acceleration from 1,500 m to 3,000 m cruise altitude
under standalone 105 kW Compression-Ignition power.
================================================================================
"""

import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.climb as cl

# ------------------------------------------------------------------------------
# 1. INITIALIZE FROM CLIMB END-OF-PHASE STATE
# ------------------------------------------------------------------------------
m_initial = float(cl.m)
m = float(cl.m)
V = float(getattr(cl, 'final_velocity', cl.V))
h = float(getattr(cl, 'final_altitude', cl.h))
s_hor = float(getattr(cl, 'cumulative_distance', cl.s_hor))

t_step = 0.1  # seconds
eta_prop = 0.74
Phase_Success = True
t = 0.0

h_target = float(ms.cruise_altitude)

# Aerodynamic target cruise speed at top of climb
guess_mass_fuel = 10.0  # kg buffer to evaluate design cruising speed
rho_cruise = float(Atmosphere(h_target).density[0])
vel_cruise = math.sqrt(
    (2.0 * (m - guess_mass_fuel) * 9.81 / (rho_cruise * uav.S))
    * math.sqrt(uav.K / uav.Cd_0)
)

# Energy-height velocity gradient dV/dh
dV_dh = (vel_cruise - V) / (h_target - h) if (h_target - h) > 0.0 else 0.0

P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW
percent_ICE_power = 1.0  # Full continuous throttle ICE

t_baseline = float(getattr(cl, 'cumulative_time', cl.cumulative_time))

# ------------------------------------------------------------------------------
# 2. TIME-MARCHING CRUISE-CLIMB INTEGRATION (1,500 m to 3,000 m MSL)
# ------------------------------------------------------------------------------
while h < h_target:
    rho = float(Atmosphere(h).density[0])

    # Dynamic altitude derating of internal combustion engine
    P_ICE_avail = uav.get_ice_available_power(h)
    P_ICE_full = max(0.0, P_ICE_avail - P_ICE_avionics)

    P_ICE = percent_ICE_power * P_ICE_full

    # Propulsion shaft and thrust power delivered to propeller (single reduction gearbox)
    P_prop_shaft = P_ICE * uav.eta_gearbox
    P_avail = P_prop_shaft * eta_prop  # kW

    # Aerodynamic drag and required power
    Cl = (m * 9.81) / (0.5 * rho * (V ** 2) * uav.S)
    Cd = uav.Cd_0 + uav.K * (Cl ** 2)
    D = 0.5 * rho * (V ** 2) * uav.S * Cd  # N
    P_req = D * V                          # W

    # Energy-height rate of climb accounting for simultaneous acceleration
    apparent_accel = 9.81 + (V * dV_dh)
    excess_power_W = (P_avail * 1000.0) - P_req
    R_C = excess_power_W / (m * apparent_accel)  # m/s

    if R_C <= 0.0:
        print(f"Cruise-Climb failed: Rate of climb collapsed to {R_C:.2f} m/s at {h:.1f} m")
        Phase_Success = False
        break

    sin_theta = max(-1.0, min(1.0, R_C / V))
    theta = math.asin(sin_theta)

    # Engine load ratio and dynamic BSFC evaluation
    P_ICE_brake_total = P_ICE + P_ICE_avionics
    xi_ICE = min(1.0, P_ICE_brake_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.0
    bsfc_cruise_climb = uav.get_ice_bsfc(P_ICE_brake_total, P_ICE_avail)  # g/kWh
    m_dot_fuel_gps = (P_ICE_brake_total * bsfc_cruise_climb) / 3600.0     # g/s

    # Mass decrement per time step (fuel consumption only)
    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0

    # State advancement
    V += R_C * dV_dh * t_step
    h += R_C * t_step
    s_hor += V * math.cos(theta) * t_step
    t += t_step
    m -= m_fuel_step_kg

    # Telemetry logging (matching Pure ICE signature)
    fuel_remaining = m - ms.m_zfw
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Cruise_Climb', h, V, P_req / 1000.0, P_ICE_brake_total,
        fuel_remaining, xi_ICE, bsfc_cruise_climb, m_dot_fuel_gps
    )

    if m < ms.m_zfw:
        print('''
        Error: Insufficient fuel remaining for mission.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 3. EXPORT VARIABLES FOR DOWNSTREAM PHASES (Phases/cruise_out.py)
# ------------------------------------------------------------------------------
cumulative_time = t_baseline + t
cumulative_distance = s_hor
final_mass = m
final_velocity = V
final_altitude = h

if Phase_Success:
    print(f'''
    ------- Cruise Climb Successful -------
    EOP ICE Total Power: {P_ICE_brake_total:.2f} kW | Load Ratio (xi): {xi_ICE:.2f}
    Propeller Shaft Power: {P_prop_shaft:.2f} kW | BSFC: {bsfc_cruise_climb:.1f} g/kWh
    Rate of Climb (EOP): {R_C:.2f} m/s
    Total Phase Climb Time: {t/60.0:.2f} mins
    Final Velocity: {V:.2f} m/s ({V*3.6:.1f} km/h) (Target: {vel_cruise:.2f} m/s)
    Horizontal Distance Travelled: {s_hor/1000.0:.2f} km
    Final Altitude: {h:.2f} m (Target: {h_target:.1f} m)
    Fuel Burned: {(m_initial - m):.2f} kg
    Usable Fuel Remaining: {fuel_remaining:.2f} kg\n''')
else:
    print(f'''
    ------- Cruise Climb Failed -------
    EOP ICE Total Power: {P_ICE_brake_total:.2f} kW
    Total Phase Climb Time: {t/60.0:.2f} mins
    Horizontal Distance: {s_hor/1000.0:.2f} km
    Final Altitude: {h:.2f} m\n''')