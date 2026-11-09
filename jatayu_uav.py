"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
MODULE: jatayu_uav.py
================================================================================
Dedicated standalone 105 kW Compression-Ignition (CI) heavy-fuel propulsion 
system. Built-in engine accessory generation. Single-input reduction gearbox 
and strict MTOW = 1000.0 kg mass closure.
================================================================================
"""

import math
import numpy as np
from ambiance import Atmosphere

# ==============================================================================
# 1. AIRFRAME GEOMETRY & FIXED MASSES
# ==============================================================================
MTOW = 1000.0              # Maximum Takeoff Weight [kg]
M_structure = 275.0        # Composite airframe, wing, empennage, landing gear [kg]
M_payload = 200.0          # Gyro-stabilized multi-sensor EO/IR payload turret [kg]
M_avionics = 45.0          # Dual-redundant FCS, navigation, transponder, datalink [kg]
M_propeller = 14.5         # Variable-pitch constant-speed composite propeller [kg]

# Powertrain Transmission
M_torsional_damper = 3.5   # Crankshaft isolator / dual-mass flywheel for CI block [kg]
M_gearbox = 8.0            # Single-input bevel-planetary propeller reduction gearbox [kg]

# Base Empty Structure (excluding engine block and fuel system)
M_fixed_empty = (
    M_structure 
    + M_payload 
    + M_avionics 
    + M_propeller 
    + M_torsional_damper 
    + M_gearbox
)  # 546.00 kg


# ==============================================================================
# 2. AERODYNAMIC POLAR & FLIGHT ENVELOPE
# ==============================================================================
b = 18.0                   # Wing Span [m]
h = 1.6                    # Tail / Fuselage Reference Height [m]
S = 22.0                   # Wing Reference Area [m^2]
lamb = 0.45                # Wing Taper Ratio
AR = (b ** 2) / S          # Wing Aspect Ratio = 14.727
delta = 0.02               # Induced drag factor
e = 1.0 / (1.0 + delta)    # Oswald span efficiency factor (e ~= 0.9804)
k1 = 0.009                 # Parasitic drag rise coefficient with CL^2
k3 = 1.0 / (math.pi * AR * e) # Induced drag polar coefficient (~0.02206)
K = k1 + k3                # Total quadratic drag factor (K ~= 0.03106)

Cd_0 = 0.0265              # Zero-lift parasitic drag (includes radiator pods & payload)
Cl_ground = 0.35           # Ground roll lift coefficient
Cl_max_ground_TO = 1.25    # Max ground-effect takeoff CL
Cl_max_TO = 1.70           # Takeoff rotation lift coefficient
Cl_max_Land = 1.85         # Flapped approach/landing lift coefficient
Cl_max_clean = 1.40        # Clean wing maximum stall lift coefficient

# Propeller Installation
D_prop = 1.75              # Propeller diameter [m]
N_prop = 2400.0            # Constant governed propeller speed [RPM]
n_prop = N_prop / 60.0     # Propeller rotational speed [rev/s] = 40.0
eta_prop = 0.66            # Flight-tested loiter efficiency (20-deg banked orbit)


# ==============================================================================
# 3. 105 kW DEDICATED CI ENGINE SPECIFICATIONS
# ==============================================================================
P_ICE_rated = 105.0        # Continuous rated brake takeoff power at MSL [kW]
N_ICE_rated = 3800.0       # Engine crankshaft speed at rated power [RPM]

# Scaled Compression-Ignition Engine Dry Core & Subsystem Accessories
# (Accessories include built-in 28V starter-generator, cooling, oil, ECU)
M_ICE_dry = 148.75         # Heavy-fuel cast block, cylinder heads, crank, pistons [kg]
M_ICE_accessories = 33.68  # Turbocharger, intercooler, radiator, oil core, internal 28V gen, exhaust [kg]
M_ICE_installed = M_ICE_dry + M_ICE_accessories  # 182.43 kg

# Transmission & Auxiliary Power
eta_gearbox = 0.98         # Direct engine-to-prop shaft mechanical efficiency
eta_alternator = 0.82      # Internal 28V generator efficiency (crankshaft to 28V bus)
P_avionics_load = 0.350    # Baseline continuous flight avionics draw [kW] (350 W)

# Dynamic BSFC Map Parameters (Heywood mass-displacement scaling applied)
# Scale factor = (60 kW / 105 kW)**0.12 = 0.9351
bsfc_scale = (60.0 / P_ICE_rated) ** 0.12
BSFC_ICE_opt = 225.0 * bsfc_scale   # ~210.40 g/kWh (Optimal cruise specific fuel burn)
BSFC_ICE_max = 245.0 * bsfc_scale   # ~229.10 g/kWh (Full-throttle WOT specific fuel burn)
BSFC_ICE_low = 340.0 * bsfc_scale   # ~317.93 g/kWh (Low part-load motoring limit)


# ==============================================================================
# 4. FUEL SYSTEM SIZING & ZERO-SUM MTOW MASS BALANCE
# ==============================================================================
k_fuel_sys = 1.05          # 5% dry mass overhead for wing bladders, lines, pumps

# Total Dry Aircraft Mass (Zero Fuel Weight excluding fuel tanks)
M_dry_empty = M_fixed_empty + M_ICE_installed  # 728.43 kg

# Remaining mass under 1000 kg MTOW dedicated to total fuel system
M_fuel_capacity_gross = MTOW - M_dry_empty             # 271.57 kg
M_fuel_usable = M_fuel_capacity_gross / k_fuel_sys     # 258.64 kg
M_fuel_tanks = M_fuel_capacity_gross - M_fuel_usable   # 12.93 kg

# Zero Fuel Weight (ZFW)
m_zfw = MTOW - M_fuel_usable                           # 741.36 kg

# Operational Fuel Reserves (6% safe reserve)
reserve_fuel_ratio = 0.06
M_fuel_reserve = M_fuel_usable * reserve_fuel_ratio    # 15.52 kg
M_fuel_mission = M_fuel_usable - M_fuel_reserve        # 243.12 kg

# Reference Ambient Density
rho_0 = 1.2250             # Sea-level standard air density [kg/m^3]

# Landing Weight Limits (STANAG 4671 / Full Abort Margin)
max_landing_mass_percent = 100.0  # % of MTOW (Permits immediate abort after takeoff)
max_landing_mass = (max_landing_mass_percent / 100.0) * MTOW  # 1000.0 kg


# ==============================================================================
# 5. GOVERNING ATMOSPHERIC & THERMODYNAMIC ENGINE METHODS
# ==============================================================================
def get_ice_available_power(h_m, P_rated=P_ICE_rated):
    """
    Computes available wide-open throttle (WOT) shaft power at altitude h_m.
    Applies the classic Gagg-Farrar turbocharged density derating equation.
    """
    atmos = Atmosphere(h_m)
    rho = float(atmos.density[0]) if hasattr(atmos.density, '__len__') else float(atmos.density)
    sigma = rho / rho_0
    factor = 1.132 * sigma - 0.132
    return max(0.0, P_rated * factor)


def get_ice_bsfc(P_brake_kW, P_avail_kW):
    """
    Brake Specific Fuel Consumption (BSFC) in [g/kWh] for the 105 kW CI block.
    Evaluates dynamic engine load ratio xi = P_brake / P_avail along a convex 
    efficiency curve anchored at optimal xi = 0.70.
    """
    if P_avail_kW <= 0.0 or P_brake_kW <= 0.0:
        return BSFC_ICE_low

    xi = float(np.clip(P_brake_kW / P_avail_kW, 0.05, 1.0))
    a = 222.2 * bsfc_scale
    b = -311.1 * bsfc_scale
    c = 333.9 * bsfc_scale
    bsfc = a * (xi ** 2) + b * xi + c
    return float(np.clip(bsfc, BSFC_ICE_opt, BSFC_ICE_low))


def get_fuel_flow_rate(P_brake_kW, P_avail_kW):
    """
    Returns instantaneous fuel consumption rate in [kg/s].
    """
    bsfc = get_ice_bsfc(P_brake_kW, P_avail_kW)
    # (P_brake [kW] * bsfc [g/kWh]) / (3600 * 1000) = kg/s
    return (P_brake_kW * bsfc) / 3600000.0