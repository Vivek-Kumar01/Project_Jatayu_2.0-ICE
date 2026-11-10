"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
PHASE 9: TERMINAL APPROACH (DECELERATION & 3-DEGREE GLIDESLOPE)
================================================================================
Flapped deceleration to V_approach followed by steady 3.0-degree glideslope descent 
to 15 m obstacle screen height under 105 kW Compression-Ignition power.
================================================================================
"""

import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.descent as ds

# ------------------------------------------------------------------------------
# 1. INITIALIZE FROM DESCENT END-OF-PHASE STATE
# ------------------------------------------------------------------------------
m_initial = float(ds.m)
m = m_initial

t_step = 0.1  # seconds
h_target = float(ms.obstacle_height)  # Screen height (15 m MSL)

# Flight idle power setting for level deceleration (6% of sea level rated power)
percent_idle_power = 6.0  # %
P_ICE_Idle = (percent_idle_power / 100.0) * uav.P_ICE_rated  # 6.30 kW for 105 kW block

# Continuous electrical parasitics drawn via engine-driven alternator
P_ICE_avionics = ms.power_avionics / uav.eta_alternator       # kW (~0.427 kW)
P_ICE_shaft = P_ICE_Idle - P_ICE_avionics                     # Net idle brake output [kW]
P_prop_shaft = max(0.0, P_ICE_shaft * uav.eta_gearbox)

eta_prop = 0.0  # Idling during clean deceleration; flat pitch / zero net positive thrust
P_avail = P_prop_shaft * eta_prop

if P_ICE_shaft < 0.0:
    print('''
WARNING: Idling power percentage is too low to power avionics alternator! Increase percent_idle_power.
''')

V = float(getattr(ds, 'final_velocity', ds.V))
h = float(getattr(ds, 'final_altitude', ds.h))

# Approach gear and flap aerodynamics
delta_Cd0_flaps = uav.Cd_0
k_uc = 3.16e-5
delta_CD_0_gear = (m * 9.81 / uav.S) * k_uc * (uav.MTOW ** -0.215)
Cd0_land = uav.Cd_0 + delta_CD_0_gear + delta_Cd0_flaps

# Approach speed threshold (1.3 * V_stall_land)
safety_margin = 1.30
rho_h = float(Atmosphere(h).density[0])
V_stall_land = math.sqrt((2.0 * m * 9.81) / (rho_h * uav.Cl_max_Land * uav.S))
V_target = V_stall_land * safety_margin

t = 0.0
s_hor = 0.0
Phase_Success = True

t_baseline = float(getattr(ds, 'cumulative_time', ds.cumulative_time))

# ------------------------------------------------------------------------------
# 2. SUB-PHASE A: LEVEL DECELERATION (V_cruise -> V_approach at 300 m AGL)
# ------------------------------------------------------------------------------
while V > V_target:
    rho = float(Atmosphere(h).density[0])
    P_ICE_avail = uav.get_ice_available_power(h)

    Cl = (m * 9.81) / (0.5 * rho * (V ** 2) * uav.S)
    Cd = Cd0_land + uav.K * (Cl ** 2)
    D = 0.5 * rho * (V ** 2) * uav.S * Cd  # N

    a_net = -D / m  # Level deceleration (gamma = 0)

    # Dynamic idle BSFC and fuel flow
    xi_ICE = min(1.0, P_ICE_Idle / P_ICE_avail) if P_ICE_avail > 0.0 else 0.05
    bsfc_idle = uav.get_ice_bsfc(P_ICE_Idle, P_ICE_avail)  # g/kWh
    m_dot_fuel_gps = (P_ICE_Idle * bsfc_idle) / 3600.0     # g/s

    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0
    m -= m_fuel_step_kg

    V += a_net * t_step
    t += t_step
    s_hor += V * t_step

    # Telemetry logging (Pure ICE signature)
    fuel_remaining = m - ms.m_zfw
    P_req = (D * V) / 1000.0  # kW
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Approach_Decel', h, V, P_req, P_ICE_Idle,
        fuel_remaining, xi_ICE, bsfc_idle, m_dot_fuel_gps
    )

    if m < ms.m_zfw:
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 3. SUB-PHASE B: GLIDESLOPE DESCENT (3.0 deg descent to 15 m Screen Height)
# ------------------------------------------------------------------------------
theta = math.radians(3.0)  # Standard 3.0-degree glideslope
ROS = V * math.sin(theta)  # Rate of sink [m/s]
eta_prop = 0.60            # Part-power approach propeller efficiency
percent_power = 0.0

while h > h_target and Phase_Success:
    rho = float(Atmosphere(h).density[0])
    P_ICE_avail = uav.get_ice_available_power(h)

    Cl = (m * 9.81) / (0.5 * rho * (V ** 2) * uav.S)
    Cd = Cd0_land + uav.K * (Cl ** 2)
    D = 0.5 * rho * (V ** 2) * uav.S * Cd  # N

    # Thrust required along the descending glideslope
    T_req = max(0.0, D - (m * 9.81 * math.sin(theta)))  # N

    # Power delivery through single reduction gearbox
    P_thrust_kW = (T_req * V) / 1000.0
    P_prop_shaft = P_thrust_kW / eta_prop
    P_ICE_prop = P_prop_shaft / uav.eta_gearbox
    P_ICE_total = P_ICE_prop + P_ICE_avionics  # Total brake power required [kW]

    percent_power = P_ICE_total / P_ICE_avail if P_ICE_avail > 0.0 else 1.0

    if percent_power > 1.0:
        print(f'''
        MISSION FAILED: Not enough power available for overcoming drag in landing approach.
        Percent Power: {percent_power*100.0:.2f} %
        ''')
        Phase_Success = False
        break

    # Dynamic glideslope BSFC and fuel flow
    xi_ICE = min(1.0, P_ICE_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.05
    bsfc_glide = uav.get_ice_bsfc(P_ICE_total, P_ICE_avail)  # g/kWh
    m_dot_fuel_gps = (P_ICE_total * bsfc_glide) / 3600.0    # g/s

    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0
    m -= m_fuel_step_kg
    h -= ROS * t_step
    s_hor += V * math.cos(theta) * t_step
    t += t_step

    # Telemetry logging (Pure ICE signature)
    fuel_remaining = m - ms.m_zfw
    P_req = (D * V) / 1000.0  # kW
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Approach_Glide', h, V, P_req, P_ICE_total,
        fuel_remaining, xi_ICE, bsfc_glide, m_dot_fuel_gps
    )

    if m < ms.m_zfw:
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 4. EXPORT VARIABLES FOR DOWNSTREAM PHASES (Phases/flare.py)
# ------------------------------------------------------------------------------
m_fuel_burned = m_initial - m
Phase_Success = Phase_Success and (h <= h_target + 0.5)

cumulative_time = t_baseline + t
cumulative_distance = float(getattr(ds, 'cumulative_distance', 0.0)) + s_hor
final_mass = m
final_velocity = V
final_altitude = h

if Phase_Success:
    print(f'''
    ------- Approach Successful -------
    Horizontal Distance Covered: {s_hor/1000.0:.2f} km
    Approach Time: {t/60.0:.2f} min
    Final Altitude: {h:.2f} m (Target: {h_target:.1f} m)
    Final Velocity: {V:.2f} m/s ({V*3.6:.1f} km/h) (Target V_app: {V_target:.2f} m/s)
    ICE Glideslope Power: {P_ICE_total:.2f} kW | Load Ratio (xi): {xi_ICE:.2f} | BSFC: {bsfc_glide:.1f} g/kWh
    Throttle Demand: {percent_power*100.0:.1f} % of available
    Final Aircraft Mass: {m:.2f} kg
    Fuel Burned: {m_fuel_burned:.2f} kg
    Usable Fuel Remaining: {fuel_remaining:.2f} kg
    Untouched Safety Reserve: {ms.mass_reserve:.2f} kg\n''')
else:
    print(f'''
    ------- Approach Failed -------
    Horizontal Distance Covered: {s_hor/1000.0:.2f} km
    Approach Time: {t/60.0:.2f} min
    Final Altitude: {h:.2f} m
    Final Mass: {m:.2f} kg\n''')