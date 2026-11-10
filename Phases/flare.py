"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
PHASE 10: FINAL APPROACH SEGMENT & FLARE MANEUVER
================================================================================
Descends from 15 m screen height to flare initiation altitude, then executes 
circular flare arc transition to touchdown at flight idle under 105 kW CI power.
================================================================================
"""

import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.approach as ap

# ------------------------------------------------------------------------------
# 1. INITIALIZE FROM APPROACH END-OF-PHASE STATE
# ------------------------------------------------------------------------------
m_initial = float(ap.m)
m = m_initial

P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW (~0.427 kW)
t_step = 0.1  # seconds

theta = float(ap.theta)
n_flare = 1.05  # Flare maneuver load factor
V_flare = 1.23 * float(ap.V_stall_land)  # Average flare speed [m/s]
R_f = (V_flare ** 2) / ((n_flare - 1.0) * 9.81)  # Flare radius of curvature [m]

h_flare = R_f * (1.0 - math.cos(theta))  # Flare initiation height AGL [m]

h = float(getattr(ap, 'final_altitude', ap.h))
V = float(getattr(ap, 'final_velocity', ap.V))

Cd0_land = float(ap.Cd0_land)
ROS = V * math.sin(theta)  # Rate of sink [m/s]
eta_prop = 0.60

s_hor_app = 0.0
t_app = 0.0
Phase_Success = True

t_baseline = float(getattr(ap, 'cumulative_time', ap.cumulative_time))
h_flare_gate = ms.altitude_takeoff + h_flare

# ------------------------------------------------------------------------------
# 2. FINAL APPROACH SEGMENT (15 m Screen Height down to Flare Initiation Height)
# ------------------------------------------------------------------------------
while h > h_flare_gate:
    rho = float(Atmosphere(h).density[0])
    P_ICE_avail = uav.get_ice_available_power(h)

    Cl = (m * 9.81) / (0.5 * rho * (V ** 2) * uav.S)
    Cd = Cd0_land + uav.K * (Cl ** 2)
    D = 0.5 * rho * (V ** 2) * uav.S * Cd

    # Thrust required along descending glideslope
    T_req = max(0.0, D - (m * 9.81 * math.sin(theta)))  # N

    # Power delivery through single reduction gearbox
    P_thrust_kW = (T_req * V) / 1000.0
    P_prop_shaft = P_thrust_kW / eta_prop
    P_ICE_prop = P_prop_shaft / uav.eta_gearbox
    P_ICE_total = P_ICE_prop + P_ICE_avionics

    percent_power = P_ICE_total / P_ICE_avail if P_ICE_avail > 0.0 else 1.0

    if percent_power > 1.0:
        print(f'''
        MISSION FAILED: Not enough power available for overcoming drag in landing.
        Percent Power: {percent_power*100.0:.2f} %
        ''')
        Phase_Success = False
        break

    # Dynamic glideslope BSFC and fuel flow
    xi_ICE = min(1.0, P_ICE_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.05
    bsfc_approach = uav.get_ice_bsfc(P_ICE_total, P_ICE_avail)  # g/kWh
    m_dot_fuel_gps = (P_ICE_total * bsfc_approach) / 3600.0     # g/s

    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0
    m -= m_fuel_step_kg
    h -= ROS * t_step
    s_hor_app += V * math.cos(theta) * t_step
    t_app += t_step

    # Telemetry logging (Pure ICE signature)
    fuel_remaining = m - ms.m_zfw
    P_req = (D * V) / 1000.0  # kW
    cumulative_time = t_baseline + t_app
    ms.log_telemetry(
        cumulative_time, 'Final_Approach', h, V, P_req, P_ICE_total,
        fuel_remaining, xi_ICE, bsfc_approach, m_dot_fuel_gps
    )

    if m < ms.m_zfw:
        print('''
        Error: Insufficient fuel remaining in approach.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 3. FLARE ARC & TOUCHDOWN KINEMATICS
# ------------------------------------------------------------------------------
percent_idle_power = 6.0  # Flight idle during circular flare maneuver
P_ICE_Idle = (percent_idle_power / 100.0) * uav.P_ICE_rated  # 6.30 kW

s_hor_flare = R_f * math.sin(theta)  # Horizontal distance traversed in flare [m]
dist_flare = R_f * theta             # Arc length [m]
t_flare = dist_flare / V_flare       # Flare duration [seconds]

# Available derated power at runway datum
P_ICE_avail_runway = uav.get_ice_available_power(ms.altitude_takeoff)
xi_ICE_flare = min(1.0, P_ICE_Idle / P_ICE_avail_runway) if P_ICE_avail_runway > 0.0 else 0.05
bsfc_flare = uav.get_ice_bsfc(P_ICE_Idle, P_ICE_avail_runway)  # g/kWh
m_dot_fuel_flare = (P_ICE_Idle * bsfc_flare) / 3600.0          # g/s

# Mass decrement during flare (fuel consumed by idling ICE)
m_fuel_flare_kg = (m_dot_fuel_flare * t_flare) / 1000.0
m -= m_fuel_flare_kg

# Touchdown parameters
V_touchdown = 1.15 * float(ap.V_stall_land)
h_touchdown = float(ms.altitude_takeoff)

# Telemetry logging at touchdown instant
fuel_remaining = m - ms.m_zfw
rho_runway = float(Atmosphere(h_touchdown).density[0])
Cl_td = (m * 9.81) / (0.5 * rho_runway * (V_touchdown ** 2) * uav.S)
Cd_td = Cd0_land + uav.K * (Cl_td ** 2)
D_flare = 0.5 * rho_runway * (V_touchdown ** 2) * uav.S * Cd_td
P_req_flare = (D_flare * V_touchdown) / 1000.0  # kW

cumulative_time = t_baseline + t_app + t_flare
ms.log_telemetry(
    cumulative_time, 'Flare_Touchdown', h_touchdown, V_touchdown, P_req_flare,
    P_ICE_Idle, fuel_remaining, xi_ICE_flare, bsfc_flare, m_dot_fuel_flare
)

if m < ms.m_zfw:
    print('''
    Error: Insufficient fuel remaining in flare.
    MISSION FAILED.
    ''')
    Phase_Success = False

# ------------------------------------------------------------------------------
# 4. EXPORT VARIABLES FOR DOWNSTREAM PHASES (Phases/touchdown.py)
# ------------------------------------------------------------------------------
m_fuel_burned = m_initial - m
final_mass = m
final_velocity = V_touchdown
final_altitude = h_touchdown
s_hor = s_hor_app + s_hor_flare
t_total_flare = t_app + t_flare

if Phase_Success:
    print(f'''
    ------- Final Approach & Flare Successful -------
    Approach Distance: {s_hor_app:.2f} m | Approach Time: {t_app:.2f} sec
    Flare Initiation Height: {h_flare:.2f} m
    Flare Ground Distance: {s_hor_flare:.2f} m | Flare Time: {t_flare:.2f} sec
    Touchdown Speed: {V_touchdown:.2f} m/s ({V_touchdown*3.6:.1f} km/h)
    Final Aircraft Mass: {m:.2f} kg
    Total Phase Fuel Burned: {m_fuel_burned*1000.0:.2f} g
    Usable Fuel Remaining: {fuel_remaining:.2f} kg
    Untouched Safety Reserve: {ms.mass_reserve:.2f} kg\n''')
else:
    print(f'''
    ------- Final Approach & Flare Failed -------
    Approach Distance: {s_hor_app:.2f} m | Time: {t_app:.2f} sec
    Final Altitude: {h:.2f} m | Final Mass: {m:.2f} kg\n''')