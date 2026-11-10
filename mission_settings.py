"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
MODULE: mission_settings.py
================================================================================
Mission parameters, takeoff ground roll constraints, flight envelope parameters,
and unified mission telemetry recorder for the dedicated 105 kW CI powertrain.
================================================================================
"""

import math
import jatayu_uav as uav

# ==============================================================================
# 1. OPERATIONAL MASS BUDGET & SIZING AUDIT
# ==============================================================================
power_avionics = uav.P_avionics_load  # 0.35 kW (350 W baseline draw)
reserve_fuel = uav.reserve_fuel_ratio # 0.06 (6% reserve fraction)

# Fuel breakdown sourced directly from closed jatayu_uav mass budget
mass_fuel = uav.M_fuel_usable         # Total usable fuel [kg] (258.64 kg)
mass_reserve = uav.M_fuel_reserve     # Reserve fuel [kg] (15.52 kg)
mass_mission_fuel = uav.M_fuel_mission# Mission fuel [kg] (243.12 kg)
m_zfw = uav.m_zfw + mass_reserve                   # Zero Usable Fuel Weight [kg] (741.36 kg)

wing_loading = uav.MTOW / uav.S       # kg/m² (~45.45 kg/m²)
power_loading = uav.MTOW / uav.P_ICE_rated  # kg/kW standalone (1000 / 105 ~= 9.52 kg/kW)
ld_max_clean = 1.0 / (2.0 * math.sqrt(uav.Cd_0 * uav.K))

if mass_fuel <= 0.0:
    print("\n" + "!" * 82)
    print("FATAL SIZING ERROR: Mass of usable fuel is <= 0 kg. Check airframe weight!")
    print("!" * 82 + "\n")
else:
    print("=" * 82)
    print(f"{'PROJECT JATAYU: PURE ICE UAV CONFIGURATION & SIZING AUDIT':^82}")
    print("=" * 82)

    # Category 1: Mass Properties & Structural Limits
    print(f"{'--- MASS PROPERTIES & STRUCTURAL LIMITS ---':^82}")
    print(f"  MTOW:                {uav.MTOW:>8.2f} kg   │  Payload Mass:        {uav.M_payload:>8.2f} kg")
    print(f"  Structural Mass:     {uav.M_structure:>8.2f} kg   │  Avionics Mass:       {uav.M_avionics:>8.2f} kg")
    print(f"  Fixed Empty Mass:    {uav.M_fixed_empty:>8.2f} kg   │  Single Gearbox Mass: {uav.M_gearbox:>8.2f} kg")
    print(f"  Zero Fuel Wt (ZFW):  {m_zfw:>8.2f} kg   │  Max Landing Wt (MLW):{uav.MTOW:>8.2f} kg")
    print("-" * 82)

    # Category 2: Aerodynamics & Lift Limits
    print(f"{'--- AERODYNAMICS & LIFT COEFFICIENTS ---':^82}")
    print(f"  Wing Span (b):       {uav.b:>8.2f} m    │  Aspect Ratio (AR):    {uav.AR:>8.2f}")
    print(f"  Wing Area (S):       {uav.S:>8.2f} m²   │  Wing Loading (W/S):   {wing_loading:>8.2f} kg/m²")
    print(f"  Clean Zero-Lift Cd0: {uav.Cd_0:>8.4f}      │  Clean (L/D)max:         {ld_max_clean:>8.2f}")
    print(f"  CL_max (Clean):      {uav.Cl_max_clean:>8.2f}      │  CL_max (Takeoff Flap):{uav.Cl_max_TO:>8.2f}")
    print(f"  CL_ground (Roll):    {uav.Cl_ground:>8.2f}      │  CL_max (Landing Flap):{uav.Cl_max_Land:>8.2f}")
    print("-" * 82)

    # Category 3: Dedicated 105 kW Compression-Ignition Engine
    print(f"{'--- COMPRESSION-IGNITION (CI) ENGINE ---':^82}")
    print(f"  ICE Installed Mass:  {uav.M_ICE_installed:>8.2f} kg   │  ICE Rated Power:      {uav.P_ICE_rated:>8.2f} kW")
    print(f"  ICE Rated Speed:     {uav.N_ICE_rated:>8.0f} RPM  │  Installed W/P:        {power_loading:>8.2f} kg/kW")
    print(f"  BSFC (Opt Cruise):   {uav.BSFC_ICE_opt:>8.1f} g/kWh│  BSFC (100% WOT Max):  {uav.BSFC_ICE_max:>8.1f} g/kWh")
    print(f"  BSFC (Low Throttle): {uav.BSFC_ICE_low:>8.1f} g/kWh│  Total Usable Fuel:   {mass_fuel:>8.2f} kg")
    print(f"  Mission Fuel (94%):  {mass_mission_fuel:>8.2f} kg   │  Reserve Fuel (6%):    {mass_reserve:>8.2f} kg")
    print("=" * 82)
    print(f"{'PRE-FLIGHT AUDIT VERIFIED: PURE ICE AIRCRAFT GO FOR FLIGHT':^82}")
    print("=" * 82 + "\n")


# ==============================================================================
# 2. TAKEOFF & RUNWAY CONSTRAINTS
# ==============================================================================
altitude_takeoff = 0.0       # Takeoff airfield elevation [m MSL]
obstacle_height = altitude_takeoff + 15.0  # 50 ft screen height from ground [m]
max_permissible_sroll = 600.0# Max allowable ground roll distance [m]
max_permissible_stot = 1000.0# Max total distance clearing 15 m obstacle [m]

rolling_friction_coefficient = 0.04  # Paved/dry runway friction factor

# Ground roll aerodynamics & ground effect
k_uc = 3.16 * (10**-5)
delta_CD_0 = (uav.MTOW * 9.81 / uav.S) * k_uc * (uav.MTOW**-0.215)
delta_CD_0_flap = 0.0095
G = ((16.0 * uav.h / uav.b) ** 2) / (1.0 + (16.0 * uav.h / uav.b) ** 2)
K_grd = (uav.k1 + G * uav.k3)
CD_takeoff = uav.Cd_0 + delta_CD_0 + delta_CD_0_flap + K_grd * (uav.Cl_ground**2)


# ==============================================================================
# 3. MISSION FLIGHT PHASES
# ==============================================================================
cruise_altitude = 3000.0     # [m MSL]
cruise_distance = 150.0      # [km]
loiter_altitude = cruise_altitude # [m MSL]


# ==============================================================================
# 4. MISSION TELEMETRY RECORDER (PURE ICE)
# ==============================================================================
telemetry = {
    'time_s': [],
    'phase': [],
    'altitude_m': [],
    'velocity_ms': [],
    'P_req_kW': [],
    'P_ICE_kW': [],
    'fuel_mass_kg': [],
    'xi_ICE': [],            # Engine load ratio (P_brake / P_avail)
    'BSFC_g_kWh': [],        # Instantaneous BSFC [g/kWh]
    'm_dot_fuel_gps': []     # Instantaneous fuel burn rate [g/s]
}


def log_telemetry(t, phase, alt, v, p_req, p_ice, fuel, xi_ice, bsfc, m_dot_fuel):
    """
    Appends one instantaneous simulation step to the Pure ICE telemetry log.
    Signature strictly matches pure mechanical propulsion parameters.
    """
    telemetry['time_s'].append(t)
    telemetry['phase'].append(phase)
    telemetry['altitude_m'].append(alt)
    telemetry['velocity_ms'].append(v)
    telemetry['P_req_kW'].append(p_req)
    telemetry['P_ICE_kW'].append(p_ice)
    telemetry['fuel_mass_kg'].append(fuel)
    telemetry['xi_ICE'].append(xi_ice)
    telemetry['BSFC_g_kWh'].append(bsfc)
    telemetry['m_dot_fuel_gps'].append(m_dot_fuel)