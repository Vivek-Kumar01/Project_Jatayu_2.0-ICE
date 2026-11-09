"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
PHASE 6: SUSTAINED ON-STATION LOITER (MAXIMUM ENDURANCE)
================================================================================
Continuous on-station loiter orbit (20 deg bank angle) under standalone 105 kW 
Compression-Ignition heavy-fuel power. Operates down to the RTB fuel floor.
================================================================================
"""

import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.cruise_out as co

# ------------------------------------------------------------------------------
# 1. LOITER ENVIRONMENTAL & AERODYNAMIC PARAMETERS
# ------------------------------------------------------------------------------
t_step = 1.0  # seconds
h = float(ms.loiter_altitude)
rho = float(Atmosphere(h).density[0])  # kg/m^3

# Standoff orbit bank angle (continuous 20 deg circle)
phi_orbit = math.radians(20.0)
n_load = 1.0 / math.cos(phi_orbit)  # n ~ 1.0642

safe_margin = 1.25  # Ratio of loiter velocity to stall velocity
cl_opt_endurance = math.sqrt(3.0 * uav.Cd_0 / uav.K)
cl_loiter = min(cl_opt_endurance, uav.Cl_max_clean / (safe_margin ** 2))

# Effective drag polar during continuous turn
cd_loiter = uav.Cd_0 + uav.K * ((cl_loiter * n_load) ** 2)
n_prop = 0.66  # Propeller efficiency accounting for turn slip & advance ratio

# Atmospheric derating for ICE at loiter altitude
P_ICE_avail = uav.get_ice_available_power(h)

# Continuous electrical parasitics drawn via engine-driven alternator
P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW

# State Initializations from Cruise-Out
m_initial = float(co.m)  # kg
m = m_initial
endurance = 0.0          # Total elapsed loiter time (s)

# Return-to-Base (RTB) fuel margin reserved for outbound-matched Cruise-In
fraction_keep = 1.05     # 5% buffer over cruise-out burn to ensure safe RTB
min_mass_cruisein = ms.m_zfw + ms.mass_reserve + (fraction_keep * co.m_fuel_burned)

Phase_Success = True
t_baseline = float(getattr(co, 'cumulative_time', co.t_baseline + co.time_cruise))

# ==============================================================================
# 2. TIME-MARCHING SUSTAINED ON-STATION LOITER (PURE ICE)
# ==============================================================================
print("--> Commencing Sustained On-Station Loiter on Pure Heavy-Fuel ICE...")

while m > min_mass_cruisein:
    # Instantaneous minimum-power loiter speed
    vel_loiter = math.sqrt(2.0 * m * 9.81 / (rho * uav.S * cl_loiter))
    D_loiter = 0.5 * rho * (vel_loiter ** 2) * uav.S * cd_loiter
    P_prop_req = (D_loiter * vel_loiter) / 1000.0  # Aerodynamic thrust power [kW]
    P_shaft_prop = P_prop_req / n_prop             # Required propeller shaft power [kW]

    # Propulsion shaft demand delivered through single reduction gearbox
    P_ICE_shaft_prop = P_shaft_prop / uav.eta_gearbox
    P_ICE_brake_total = P_ICE_shaft_prop + P_ICE_avionics

    if P_ICE_brake_total > P_ICE_avail:
        Phase_Success = False
        print("Error: Required ICE power exceeds available derated power during loiter.")
        break

    # Load ratio and dynamic BSFC evaluation in loiter regime
    xi_ICE = min(1.0, P_ICE_brake_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.0
    bsfc_loiter = uav.get_ice_bsfc(P_ICE_brake_total, P_ICE_avail)  # g/kWh
    m_dot_fuel_gps = (P_ICE_brake_total * bsfc_loiter) / 3600.0     # g/s

    # Mass decrement per step (fuel consumption only)
    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0
    m -= m_fuel_step_kg

    if m <= (ms.m_zfw + ms.mass_reserve):
        print("Emergency fuel reserve breached during loiter! RTB immediately.")
        break

    endurance += t_step

    # Telemetry logging (Pure ICE signature)
    fuel_remaining = m - ms.m_zfw
    ms.log_telemetry(
        t_baseline + endurance, 'Sustained_Loiter', h,
        vel_loiter, P_prop_req, P_ICE_brake_total,
        fuel_remaining, xi_ICE, bsfc_loiter, m_dot_fuel_gps
    )

# ------------------------------------------------------------------------------
# 3. EXPORT VARIABLES FOR DOWNSTREAM PHASES (Phases/cruise_in.py)
# ------------------------------------------------------------------------------
m_fuel_loiter_burned = m_initial - m
cumulative_time = t_baseline + endurance
final_mass = m
final_velocity = vel_loiter
final_altitude = h

if Phase_Success:
    print(f'''
    ------- Sustained Loiter Successful (Pure ICE 105 kW) -------
    Total Loiter Endurance: {endurance / 3600.0:.2f} hours ({(endurance / 60.0):.1f} mins)
    Loiter Speed (Final): {vel_loiter:.2f} m/s ({vel_loiter*3.6:.1f} km/h)
    Propeller Shaft Power: {P_shaft_prop:.2f} kW
    ICE Total Brake Power: {P_ICE_brake_total:.2f} kW | Load Ratio (xi): {xi_ICE:.2f}
    Instantaneous BSFC: {bsfc_loiter:.1f} g/kWh | Burn Rate: {m_dot_fuel_gps:.2f} g/s
    Fuel Burned During Loiter: {m_fuel_loiter_burned:.2f} kg
    Final Aircraft Mass: {m:.2f} kg
    Usable Fuel Remaining for RTB: {fuel_remaining - ms.mass_reserve:.2f} kg
    Untouched Reserve Fuel: {ms.mass_reserve:.2f} kg\n''')
else:
    print(f'''
    ------- Loiter Failed -------
    Total Loiter Endurance: {endurance / 3600.0:.2f} hours
    Final Mass: {m:.2f} kg\n''')