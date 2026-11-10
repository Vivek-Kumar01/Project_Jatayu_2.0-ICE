"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
PHASE 2: PULL-UP & CLIMB TO SCREEN HEIGHT (15 m OBSTACLE)
================================================================================
Climb from rotation to 15 m obstacle clearance under standalone 105 kW CI power.
================================================================================
"""

import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.takeoff as to

# ------------------------------------------------------------------------------
# 1. INITIALIZE FROM TAKEOFF END-OF-PHASE STATE
# ------------------------------------------------------------------------------
m_initial = float(to.m)
m = float(to.m)
h = float(to.h)
V = float(getattr(to, 'final_velocity', to.V_LO))
s_roll = float(to.s_roll)

t_step = 0.1  # seconds
theta = 6.5   # Climb angle [degrees]
Cd_0_p1 = uav.Cd_0 + ms.delta_CD_0 + ms.delta_CD_0_flap
eta_prop = 0.68

# ------------------------------------------------------------------------------
# 2. POWERTRAIN RATING & DERATING AT SCREEN HEIGHT
# ------------------------------------------------------------------------------
rho = float(Atmosphere(h).density[0])  # kg/m^3
P_ICE_avail = uav.get_ice_available_power(h)

# Continuous electrical parasitics drawn via engine-driven alternator
P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW
P_ICE_full = max(0.0, P_ICE_avail - P_ICE_avionics)     # kW

# 100% full throttle ICE climb
percent_ICE_power = 1.0
P_ICE = percent_ICE_power * P_ICE_full

# Drivetrain mechanical delivery through single reduction gearbox
P_shaft_prop = P_ICE * uav.eta_gearbox
P_available = P_shaft_prop * eta_prop  # kW (thrust power)

# ------------------------------------------------------------------------------
# 3. ENGINE LOAD & FUEL CONSUMPTION METRICS
# ------------------------------------------------------------------------------
P_ICE_brake_total = P_ICE + P_ICE_avionics  # Total crankshaft brake demand [kW]
xi_ICE = min(1.0, P_ICE_brake_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.0
bsfc_pullup = uav.get_ice_bsfc(P_ICE_brake_total, P_ICE_avail)  # g/kWh
m_dot_fuel_gps = (P_ICE_brake_total * bsfc_pullup) / 3600.0     # g/s

# ------------------------------------------------------------------------------
# 4. TIME-MARCHING PULL-UP INTEGRATION (Climb to 15 m Obstacle Height)
# ------------------------------------------------------------------------------
t = 0.0
Phase_Success = True

while h < ms.obstacle_height:
    # Update atmospheric density with altitude
    rho = float(Atmosphere(h).density[0])

    L = m * 9.81 * math.cos(math.radians(theta))  # N
    Cl = L / (0.5 * rho * (V ** 2) * uav.S)
    Cd = Cd_0_p1 + uav.K * (Cl ** 2)
    D = 0.5 * rho * (V ** 2) * uav.S * Cd         # N
    T = (P_available * 1000.0) / V                # N
    F_net = T - D - (m * 9.81 * math.sin(math.radians(theta)))  # N
    a_net = F_net / m                             # m/s^2

    # Fuel mass consumed in step (propulsion + avionics alternator)
    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0  # kg

    # State advancement
    V += a_net * t_step
    h += V * math.sin(math.radians(theta)) * t_step
    t += t_step
    s_roll += V * math.cos(math.radians(theta)) * t_step
    m -= m_fuel_step_kg

    # Telemetry logging (matching Pure ICE signature)
    fuel_remaining = m - ms.m_zfw
    P_req = (D * V) / 1000.0  # kW
    cumulative_time = float(getattr(to, 'cumulative_time', to.t + to.N)) + t
    ms.log_telemetry(
        cumulative_time, 'Pull_Up', h, V, P_req, P_ICE_brake_total,
        fuel_remaining, xi_ICE, bsfc_pullup, m_dot_fuel_gps
    )

    if h < ms.obstacle_height and s_roll > ms.max_permissible_stot:
        print('''
        Error: Horizontal distance exceeded maximum permissible limit to 15 m obstacle.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    if m < ms.m_zfw:
        print('''
        Error: Insufficient fuel remaining for mission.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 5. EXPORT VARIABLES FOR DOWNSTREAM PHASES (Phases/climb.py)
# ------------------------------------------------------------------------------
cumulative_time = float(getattr(to, 'cumulative_time', to.t + to.N)) + t
cumulative_distance = s_roll
final_mass = m
final_velocity = V
final_altitude = h

if Phase_Success:
    print(f'''
    ------- Pull Up Successful -------
    ICE Total Shaft Power: {P_ICE_brake_total:.2f} kW | Load Ratio (xi): {xi_ICE:.2f}
    Propeller Shaft Power: {P_shaft_prop:.2f} kW | BSFC: {bsfc_pullup:.1f} g/kWh
    Total Phase Climb Time: {t:.2f} seconds
    Total Distance to Clear Obstacle: {s_roll:.2f} meters (Limit: {ms.max_permissible_stot:.0f} m)
    Final Velocity: {V:.2f} m/s ({V*3.6:.1f} km/h)
    Final Altitude: {h:.2f} meters
    Fuel Burned: {(m_initial - m)*1000.0:.2f} g (Rate: {m_dot_fuel_gps:.2f} g/s)
    Usable Fuel Remaining: {fuel_remaining:.2f} kg\n''')
else:
    print(f'''
    ------- Pull Up Failed -------
    ICE Total Shaft Power: {P_ICE_brake_total:.2f} kW
    Total Phase Climb Time: {t:.2f} seconds
    Total Distance: {s_roll:.2f} meters
    Final Speed: {V:.2f} m/s | Altitude: {h:.2f} m\n''')