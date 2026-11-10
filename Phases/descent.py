"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
PHASE 8: GLIDE DESCENT (3,000 m TO 300 m APPROACH GATE)
================================================================================
Unpowered aerodynamic glide descent down to the approach fix with the 105 kW 
Compression-Ignition engine operating at flight idle. Direct-driven 28V alternator 
supplies avionics electrical load.
================================================================================
"""

import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.cruise_in as ci

# ------------------------------------------------------------------------------
# 1. INITIALIZE FROM CRUISE-IN END-OF-PHASE STATE
# ------------------------------------------------------------------------------
m_initial = float(ci.m)
m = m_initial

t_step = 0.1  # seconds
h_target = ms.altitude_takeoff + 300.0  # m (Approach gate at 300 m AGL)

# Flight idle power setting (6% of sea level rated power)
percent_idle_power = 6.0  # %
P_ICE_Idle = (percent_idle_power / 100.0) * uav.P_ICE_rated  # 6.30 kW for 105 kW block

# Direct mechanical parasitic draw for 350 W avionics via built-in alternator
P_ICE_avionics = ms.power_avionics / uav.eta_alternator       # ~0.427 kW
P_ICE_shaft = P_ICE_Idle - P_ICE_avionics                     # Net surplus idle brake power [kW]
P_prop_shaft = max(0.0, P_ICE_shaft * uav.eta_gearbox)

eta_prop = 0.0  # Flight idle: windmilling / flat pitch, zero net positive thrust
P_avail = P_prop_shaft * eta_prop

if P_ICE_shaft < 0.0:
    print('''
WARNING: Idling power percentage is too low to power avionics alternator! Increase percent_idle_power.
''')

V = float(getattr(ci, 'final_velocity', ci.vel_cruise))
t = 0.0
s_hor = 0.0
h = float(getattr(ci, 'final_altitude', ms.cruise_altitude))

Phase_Success = True
t_baseline = float(getattr(ci, 'cumulative_time', ci.cumulative_time))

# ------------------------------------------------------------------------------
# 2. TIME-MARCHING DESCENT INTEGRATION (Cruise Altitude to 300 m)
# ------------------------------------------------------------------------------
while h > h_target:
    rho = float(Atmosphere(h).density[0])  # kg/m^3

    # Dynamic derated capacity at instantaneous descent altitude
    P_ICE_avail = uav.get_ice_available_power(h)

    # Aerodynamics at glide/descent attitude
    Cl = (m * 9.81) / (0.5 * rho * (V ** 2) * uav.S)
    Cd = uav.Cd_0 + uav.K * (Cl ** 2)
    D = 0.5 * rho * (V ** 2) * uav.S * Cd
    P_req = D * V  # W

    # Rate of sink (unpowered glide equilibrium)
    ROS = P_req / (m * 9.81)  # m/s
    sin_theta = max(-1.0, min(1.0, ROS / V))
    theta = math.asin(sin_theta)

    # Dynamic idle BSFC and fuel flow
    xi_ICE = min(1.0, P_ICE_Idle / P_ICE_avail) if P_ICE_avail > 0.0 else 0.05
    bsfc_descent = uav.get_ice_bsfc(P_ICE_Idle, P_ICE_avail)  # g/kWh
    m_dot_fuel_gps = (P_ICE_Idle * bsfc_descent) / 3600.0     # g/s

    # Mass decrement per step (fuel consumed by idling ICE)
    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0
    m -= m_fuel_step_kg

    # State advancement
    h -= ROS * t_step
    s_hor += V * math.cos(theta) * t_step
    t += t_step

    # Telemetry logging (Pure ICE signature)
    fuel_remaining = m - ms.m_zfw
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Descent', h, V, P_req / 1000.0, P_ICE_Idle,
        fuel_remaining, xi_ICE, bsfc_descent, m_dot_fuel_gps
    )

    if m < ms.m_zfw:
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 3. EXPORT VARIABLES FOR DOWNSTREAM PHASES (Phases/approach.py)
# ------------------------------------------------------------------------------
m_fuel_burned = m_initial - m
Phase_Success = Phase_Success and (h <= h_target + 1.0)

cumulative_time = t_baseline + t
cumulative_distance = float(getattr(ci, 'cumulative_distance', 0.0)) + s_hor
final_mass = m
final_velocity = V
final_altitude = h

if Phase_Success:
    print(f'''
    ------- Descent Successful (Pure ICE 105 kW) -------
    Horizontal Distance Covered: {s_hor/1000.0:.2f} km
    Descent Time: {t/60.0:.2f} min
    Final Altitude: {h:.2f} m (Target: {h_target:.1f} m)
    Final Velocity: {V:.2f} m/s ({V*3.6:.1f} km/h)
    ICE Flight Idle: {P_ICE_Idle:.2f} kW | Load Ratio (xi): {xi_ICE:.2f} | BSFC: {bsfc_descent:.1f} g/kWh
    Final Mass: {m:.2f} kg
    Fuel Burned: {m_fuel_burned:.2f} kg (Burn Rate: {m_dot_fuel_gps:.2f} g/s)
    Usable Fuel Remaining: {fuel_remaining:.2f} kg
    Untouched Safety Reserve: {ms.mass_reserve:.2f} kg\n''')
else:
    print(f'''
    ------- Descent Failed -------
    Horizontal Distance Covered: {s_hor/1000.0:.2f} km
    Descent Time: {t/60.0:.2f} min
    Final Altitude: {h:.2f} m
    Final Mass: {m:.2f} kg\n''')