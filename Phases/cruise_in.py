"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
PHASE 7: INBOUND RETURN CRUISE (STATION TO DESCENT GATE)
================================================================================
Direct return transit from on-station loiter to top-of-descent gate (50 km from base)
under standalone 105 kW Compression-Ignition power.
================================================================================
"""

import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.loiter as lo

# ------------------------------------------------------------------------------
# 1. INITIALIZE FROM LOITER END-OF-PHASE STATE
# ------------------------------------------------------------------------------
descent_dist = 50.0  # km descent gate distance from base
t_step = 1.0         # seconds
h = float(ms.cruise_altitude)
rho = float(Atmosphere(h).density[0])  # kg/m^3

# Available ICE brake power at cruise altitude
P_ICE_avail = uav.get_ice_available_power(h)

# Continuous electrical parasitics drawn via engine-driven alternator
P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW
P_ICE_full = max(0.0, P_ICE_avail - P_ICE_avionics)     # kW

n_prop = 0.76  # Propeller efficiency in steady cruise

dist_covered = 0.0  # km
m_initial = float(lo.m)
m = m_initial
time_cruise = 0.0  # seconds

cl_cruise = math.sqrt(uav.Cd_0 / uav.K)
cd_cruise = uav.Cd_0 + uav.K * (cl_cruise ** 2)
Phase_Success = True

t_baseline = float(getattr(lo, 'cumulative_time', lo.t_baseline + lo.endurance))

# Direct return transit: Station straight to descent gate (150 km - 50 km = 100 km)
target_transit_dist = ms.cruise_distance - descent_dist

# ------------------------------------------------------------------------------
# 2. TIME-MARCHING CRUISE-IN INTEGRATION (Direct Return Transit)
# ------------------------------------------------------------------------------
while dist_covered < target_transit_dist:
    # Instantaneous best-range airspeed and required aerodynamic thrust power
    vel_cruise = math.sqrt((2.0 * m * 9.81 / (rho * uav.S)) * math.sqrt(uav.K / uav.Cd_0))  # m/s
    D_cruise = 0.5 * rho * (vel_cruise ** 2) * uav.S * cd_cruise
    P_prop_req = (D_cruise * vel_cruise) / 1000.0  # Aerodynamic thrust power [kW]
    P_shaft_prop = P_prop_req / n_prop             # Required propeller shaft power [kW]

    # Propulsion shaft demand delivered through single reduction gearbox
    P_ICE_prop = P_shaft_prop / uav.eta_gearbox

    # Check against available continuous ICE brake power
    if P_ICE_prop > P_ICE_full:
        print('''
        Error: Required propeller shaft power exceeds available ICE power.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    # Total engine crankshaft brake load (Propulsion + Avionics Alternator)
    P_ICE_brake_total = P_ICE_prop + P_ICE_avionics
    xi_ICE = min(1.0, P_ICE_brake_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.0
    bsfc_cruise_in = uav.get_ice_bsfc(P_ICE_brake_total, P_ICE_avail)  # g/kWh
    m_dot_fuel_gps = (P_ICE_brake_total * bsfc_cruise_in) / 3600.0     # g/s

    # Mass decrement per step (fuel consumption only)
    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0
    m -= m_fuel_step_kg

    if m < ms.m_zfw:
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    dist_covered += (vel_cruise * t_step) / 1000.0  # km
    time_cruise += t_step                          # seconds

    # Telemetry logging (Pure ICE signature)
    fuel_remaining = m - ms.m_zfw
    cumulative_time = t_baseline + time_cruise
    ms.log_telemetry(
        cumulative_time, 'Cruise_In', h, vel_cruise, P_prop_req, P_ICE_brake_total,
        fuel_remaining, xi_ICE, bsfc_cruise_in, m_dot_fuel_gps
    )

# ------------------------------------------------------------------------------
# 3. EXPORT VARIABLES FOR DOWNSTREAM PHASES (Phases/descent.py)
# ------------------------------------------------------------------------------
m_fuel_burned = m_initial - m
Phase_Success = Phase_Success and (dist_covered >= target_transit_dist)

cumulative_time = t_baseline + time_cruise
final_mass = m
final_velocity = vel_cruise
final_altitude = h

if Phase_Success:
    print(f'''
    ------- Cruise In Successful -------
    Target Transit Distance: {target_transit_dist:.2f} km | Covered: {dist_covered:.2f} km
    Transit Time: {time_cruise/60.0:.2f} min ({time_cruise/3600.0:.2f} hours)
    Cruise Velocity: {vel_cruise:.2f} m/s ({vel_cruise*3.6:.1f} km/h)
    Propeller Shaft Power: {P_shaft_prop:.2f} kW
    ICE Total Brake Load: {P_ICE_brake_total:.2f} kW | Load Ratio (xi): {xi_ICE:.2f} | BSFC: {bsfc_cruise_in:.1f} g/kWh
    Final Aircraft Mass: {m:.2f} kg
    Fuel Burned: {m_fuel_burned:.2f} kg (Burn Rate: {m_dot_fuel_gps:.2f} g/s)
    Usable Fuel Remaining: {fuel_remaining:.2f} kg
    Untouched Safety Reserve: {ms.mass_reserve:.2f} kg\n''')
else:
    print(f'''
    ------- Cruise In Failed -------
    Distance Covered: {dist_covered:.2f} km (Target: {target_transit_dist:.2f} km)
    Time: {time_cruise/60.0:.2f} min
    Final Mass: {m:.2f} kg\n''')