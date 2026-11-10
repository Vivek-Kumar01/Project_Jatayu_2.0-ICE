"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
PHASE 1: TAKEOFF GROUND ROLL & ROTATION DYNAMICS
================================================================================
Standalone 105 kW Compression-Ignition engine ground run and rotation.
================================================================================
"""

import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms

# ------------------------------------------------------------------------------
# 1. INITIAL RUNWAY & ENVIRONMENTAL CONDITIONS
# ------------------------------------------------------------------------------
h = ms.altitude_takeoff
rho = float(Atmosphere(h).density[0])  # kg/m^3
t_step = 0.1  # seconds

# Available engine brake power at runway altitude
P_ICE_avail = uav.get_ice_available_power(h)

# Continuous electrical parasitics drawn via engine-driven alternator
P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW

# Net mechanical power available for propeller shaft after alternator parasitic
P_ICE_full = max(0.0, P_ICE_avail - P_ICE_avionics)  # kW

# Throttle setting for maximum takeoff performance (100% full throttle)
percent_ICE_power = 1.0
P_ICE = percent_ICE_power * P_ICE_full  # Propulsion brake shaft power [kW]

# Drivetrain mechanical delivery through single reduction gearbox
P_shaft_prop = P_ICE * uav.eta_gearbox  # kW

# ------------------------------------------------------------------------------
# 2. INSTANTANEOUS ENGINE LOAD & FUEL CONSUMPTION METRICS
# ------------------------------------------------------------------------------
P_ICE_brake_total = P_ICE + P_ICE_avionics  # Total crankshaft brake demand [kW]
xi_ICE = min(1.0, P_ICE_brake_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.0
bsfc_takeoff = uav.get_ice_bsfc(P_ICE_brake_total, P_ICE_avail)  # g/kWh
m_dot_fuel_gps = (P_ICE_brake_total * bsfc_takeoff) / 3600.0     # g/s

# ------------------------------------------------------------------------------
# 3. AERODYNAMIC & GROUND ROLL THRESHOLDS
# ------------------------------------------------------------------------------
Merit_Factor = 0.7  # Static propeller figure of merit
Static_thrust = (Merit_Factor ** (2 / 3)) * (
    2.0 * rho * (math.pi / 4.0 * (uav.D_prop ** 2)) * ((P_shaft_prop * 1000.0) ** 2)
) ** (1 / 3)

N = 1.0  # Rotation transition duration [seconds]
V_stall = math.sqrt((2.0 * uav.MTOW * 9.81) / (rho * uav.S * uav.Cl_max_ground_TO))  # m/s
V_LO = 1.15 * V_stall  # Liftoff speed (1.15 * V_stall) [m/s]

prop_eff_liftoff = 0.65
Thrust_liftoff = (prop_eff_liftoff * P_shaft_prop * 1000.0) / V_LO  # N

def thrust_available(V):
    """Quadratic decay of static thrust towards high-speed advance."""
    return Static_thrust + (Thrust_liftoff - Static_thrust) * ((V / V_LO) ** 2)

# ------------------------------------------------------------------------------
# 4. TIME-MARCHING GROUND ROLL INTEGRATION
# ------------------------------------------------------------------------------
t = 0.0        # seconds
s_roll = 0.0   # m
V = 0.0        # m/s
m = float(uav.MTOW)  # kg

while V < V_LO:
    T = thrust_available(V)
    D = 0.5 * rho * (V ** 2) * uav.S * ms.CD_takeoff
    W = m * 9.81
    L = 0.5 * rho * (V ** 2) * uav.S * uav.Cl_ground
    
    F_net = T - D - (W - L) * ms.rolling_friction_coefficient
    a_net = F_net / m

    # Fuel mass consumed in step (propulsion + avionics alternator)
    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0  # kg

    # Kinematics and mass updates
    s_roll += V * t_step
    V += a_net * t_step
    t += t_step
    m -= m_fuel_step_kg

    # Telemetry logging (matching Pure ICE signature)
    fuel_remaining = m - ms.m_zfw
    P_req = (D * V) / 1000.0  # kW
    ms.log_telemetry(
        t, 'Takeoff', h, V, P_req, P_ICE_brake_total,
        fuel_remaining, xi_ICE, bsfc_takeoff, m_dot_fuel_gps
    )

    if s_roll > ms.max_permissible_sroll:
        print("Takeoff failed: maximum permissible ground roll exceeded.")
        break

    if m < ms.m_zfw:
        print("Takeoff failed: insufficient fuel remaining.")
        break

# ------------------------------------------------------------------------------
# 5. ROTATION SEGMENT DYNAMICS (N seconds)
# ------------------------------------------------------------------------------
s_roll += N * V_LO
t += N

# Fuel consumed during rotation
m_fuel_rotation_kg = (m_dot_fuel_gps * N) / 1000.0
m -= m_fuel_rotation_kg

fuel_remaining = m - ms.m_zfw
P_req_rotation = (0.5 * rho * (V_LO ** 2) * uav.S * ms.CD_takeoff * V_LO) / 1000.0  # kW

ms.log_telemetry(
    t, 'Takeoff_Rotation', h, V_LO, P_req_rotation, P_ICE_brake_total,
    fuel_remaining, xi_ICE, bsfc_takeoff, m_dot_fuel_gps
)

# ------------------------------------------------------------------------------
# 6. EXPORT VARIABLES FOR DOWNSTREAM PHASES
# ------------------------------------------------------------------------------
Phase_Success = (s_roll <= ms.max_permissible_sroll) and (m >= ms.m_zfw)

cumulative_time = t
cumulative_distance = s_roll
final_mass = m
final_velocity = V_LO
final_altitude = h

if not Phase_Success:
    print(f"Takeoff failed: ground roll {s_roll:.1f} m exceeds limit {ms.max_permissible_sroll:.1f} m")
else:
    print(f'''
    ------- Takeoff Successful -------
    ICE Total Shaft Power: {P_ICE_brake_total:.2f} kW | Load Ratio (xi): {xi_ICE:.2f}
    Propeller Shaft Power: {P_shaft_prop:.2f} kW | BSFC: {bsfc_takeoff:.1f} g/kWh
    Total Ground Roll Distance: {s_roll:.2f} m (Limit: {ms.max_permissible_sroll:.0f} m)
    Liftoff Velocity (V_LO): {V_LO:.2f} m/s ({V_LO*3.6:.1f} km/h)
    Time to Liftoff: {t:.2f} seconds
    Fuel Burned: {(uav.MTOW - m)*1000.0:.2f} g (Rate: {m_dot_fuel_gps:.2f} g/s)
    Usable Fuel Remaining: {fuel_remaining:.2f} kg\n''')