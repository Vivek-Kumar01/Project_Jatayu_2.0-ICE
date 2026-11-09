"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
MODULE: ice_fuel_sweep.py
================================================================================
Parametric Trade-off: ICE Engine Sizing vs. Usable Heavy Fuel Mass under 
Fixed MTOW (1,000 kg). Evaluates Heywood BSFC scaling, takeoff ground roll 
limits, and on-station loiter endurance.
================================================================================
"""

import os
import math
import importlib
from contextlib import redirect_stdout
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ------------------------------------------------------------------------------
# 0. SUPPRESS INITIAL PRINTOUTS ON IMPORT
# ------------------------------------------------------------------------------
with open(os.devnull, 'w', encoding='utf-8') as fnull:
    with redirect_stdout(fnull):
        import jatayu_uav as uav
        import mission_settings as ms

PHASE_MODULE_NAMES = [
    'Phases.takeoff',
    'Phases.pullup',
    'Phases.climb',
    'Phases.cruise_climb',
    'Phases.cruise_out',
    'Phases.loiter',
    'Phases.cruise_in',
    'Phases.descent',
    'Phases.approach',
    'Phases.flare',
    'Phases.touchdown'
]

# Baseline anchors at 105 kW design point
P_ICE_BASE = 105.0  # kW
M_ICE_INST_BASE = float(uav.M_ICE_dry + uav.M_ICE_accessories)  # 182.43 kg
BSFC_SCALE_REF = (60.0 / P_ICE_BASE) ** 0.12

# Sweep ICE rated power from 60 kW (extreme lightweight) to 140 kW (heavy high-power)
P_ICE_SWEEP = np.linspace(60.0, 140.0, 17)
sweep_records = []

print("=" * 102)
print(f"{'PROJECT JATAYU: ICE SIZING vs. FUEL MASS TRADE STUDY (MTOW = 1000 kg)':^102}")
print("=" * 102)

for idx, p_ice in enumerate(P_ICE_SWEEP):
    # --------------------------------------------------------------------------
    # 1. COMPONENT MASS SCALING & ZERO-SUM FUEL MASS BALANCE
    # --------------------------------------------------------------------------
    # Linear mass scaling of core block and accessories
    m_ice_installed = M_ICE_INST_BASE * (p_ice / P_ICE_BASE)
    
    # Airframe fixed empty mass (excluding engine and fuel system)
    m_fixed_empty = (
        uav.M_structure 
        + uav.M_payload 
        + uav.M_avionics 
        + uav.M_propeller 
        + uav.M_torsional_damper 
        + uav.M_gearbox
    )
    
    # Available fuel capacity under rigid MTOW = 1000 kg closure
    m_dry_empty = m_fixed_empty + m_ice_installed
    m_fuel_capacity_gross = uav.MTOW - m_dry_empty
    
    if m_fuel_capacity_gross <= 0.0:
        print(f"[{idx+1:02d}/{len(P_ICE_SWEEP):02d}] P_ICE: {p_ice:5.1f} kW --> DISCARDED (Airframe mass exceeds 1000 kg MTOW)")
        continue

    m_fuel_usable = m_fuel_capacity_gross / uav.k_fuel_sys
    m_fuel_reserve = m_fuel_usable * uav.reserve_fuel_ratio
    m_fuel_mission = m_fuel_usable - m_fuel_reserve
    m_zfw = uav.MTOW - m_fuel_usable

    # Apply mutated mass and power variables to modules
    uav.P_ICE_rated = float(p_ice)
    uav.M_ICE_installed = float(m_ice_installed)
    uav.M_fuel_usable = float(m_fuel_usable)
    uav.M_fuel_reserve = float(m_fuel_reserve)
    uav.M_fuel_mission = float(m_fuel_mission)
    uav.m_zfw = float(m_zfw)

    ms.mass_fuel = float(m_fuel_usable)
    ms.mass_reserve = float(m_fuel_reserve)
    ms.mass_mission_fuel = float(m_fuel_mission)
    ms.m_zfw = float(m_zfw)

    # --------------------------------------------------------------------------
    # 2. THERMODYNAMIC BSFC MAP SCALING (Heywood Exponent)
    # --------------------------------------------------------------------------
    bsfc_scale = (60.0 / p_ice) ** 0.12
    uav.bsfc_scale = bsfc_scale
    uav.BSFC_ICE_opt = 225.0 * bsfc_scale
    uav.BSFC_ICE_max = 245.0 * bsfc_scale
    uav.BSFC_ICE_low = 340.0 * bsfc_scale

    # Dynamically bind scaled BSFC polynomial
    def make_scaled_bsfc_func(scale):
        def dynamic_bsfc(P_brake_kW, P_avail_kW):
            if P_avail_kW <= 0.0 or P_brake_kW <= 0.0:
                return 340.0 * scale
            xi = float(np.clip(P_brake_kW / P_avail_kW, 0.05, 1.0))
            bsfc = (222.2 * (xi ** 2) - 311.1 * xi + 333.9) * scale
            return float(np.clip(bsfc, 225.0 * scale, 340.0 * scale))
        return dynamic_bsfc

    uav.get_ice_bsfc = make_scaled_bsfc_func(bsfc_scale)

    # --------------------------------------------------------------------------
    # 3. RESET TELEMETRY BUFFER (Pure ICE 10-Channel Schema)
    # --------------------------------------------------------------------------
    ms.telemetry = {
        'time_s': [], 'phase': [], 'altitude_m': [], 'velocity_ms': [],
        'P_req_kW': [], 'P_ICE_kW': [], 'fuel_mass_kg': [],
        'xi_ICE': [], 'BSFC_g_kWh': [], 'm_dot_fuel_gps': []
    }

    # --------------------------------------------------------------------------
    # 4. EXECUTE PHASES SILENTLY & VALIDATE FEASIBILITY
    # --------------------------------------------------------------------------
    sim_failed = False
    failed_phase_name = None

    with open(os.devnull, 'w', encoding='utf-8') as fnull:
        with redirect_stdout(fnull):
            for mod_name in PHASE_MODULE_NAMES:
                try:
                    mod = importlib.import_module(mod_name)
                    importlib.reload(mod)

                    phase_ok = getattr(mod, 'Phase_Success', True)
                    if not phase_ok:
                        sim_failed = True
                        failed_phase_name = mod_name
                        break

                except Exception as e:
                    sim_failed = True
                    failed_phase_name = f"{mod_name} (Crash: {e})"
                    break

    if sim_failed:
        print(f"[{idx+1:02d}/{len(P_ICE_SWEEP):02d}] P_ICE: {p_ice:5.1f} kW | Fuel: {m_fuel_usable:5.1f} kg --> DISCARDED (Failed in {failed_phase_name})")
        continue

    # --------------------------------------------------------------------------
    # 5. EXTRACT PERFORMANCE METRICS
    # --------------------------------------------------------------------------
    df_sim = pd.DataFrame(ms.telemetry)
    total_hr = df_sim['time_s'].iloc[-1] / 3600.0

    df_to = df_sim[df_sim['phase'] == 'Takeoff']
    s_roll_m = getattr(mod_to := importlib.import_module('Phases.takeoff'), 's_roll', 0.0)

    df_loiter = df_sim[df_sim['phase'] == 'Sustained_Loiter']
    t_loiter_hr = ((df_loiter['time_s'].iloc[-1] - df_loiter['time_s'].iloc[0]) / 3600.0) if not df_loiter.empty else 0.0

    df_ice = df_sim[df_sim['P_ICE_kW'] > 0.5]
    avg_bsfc = df_ice['BSFC_g_kWh'].mean() if not df_ice.empty else 0.0

    sweep_records.append({
        'P_ICE_kW': p_ice,
        'M_ICE_kg': m_ice_installed,
        'M_Fuel_kg': m_fuel_usable,
        'Ground_Roll_m': s_roll_m,
        'BSFC_opt_g_kWh': uav.BSFC_ICE_opt,
        'Mean_BSFC_g_kWh': avg_bsfc,
        'Sustained_Loiter_hr': t_loiter_hr,
        'Total_Endurance_hr': total_hr
    })

    print(f"[{idx+1:02d}/{len(P_ICE_SWEEP):02d}] P_ICE: {p_ice:5.1f} kW | ICE Mass: {m_ice_installed:5.1f} kg | "
          f"Fuel: {m_fuel_usable:5.1f} kg | S_roll: {s_roll_m:5.1f} m | Loiter: {t_loiter_hr:5.2f} hr | BSFC_opt: {uav.BSFC_ICE_opt:5.1f}")

# ------------------------------------------------------------------------------
# 6. EXPORT CSV RESULTS
# ------------------------------------------------------------------------------
if not sweep_records:
    print("\nError: All design points failed flight constraints.")
    exit(1)

df_res = pd.DataFrame(sweep_records)
df_res.to_csv('ice_fuel_trade_study_results.csv', index=False)
print("\n--> Results exported to ice_fuel_trade_study_results.csv")

# ------------------------------------------------------------------------------
# 7. GENERATE DUAL-AXIS TRADE-OFF VISUALIZATION
# ------------------------------------------------------------------------------
fig, ax1 = plt.subplots(figsize=(10.5, 6.5), dpi=300)

color_loiter = '#1F78B4'   # Navy Blue (Endurance)
color_roll   = '#D95F02'   # Rust/Orange (Takeoff Runway Distance)

# Primary Left Axis: Sustained Loiter Endurance
line1 = ax1.plot(
    df_res['P_ICE_kW'],
    df_res['Sustained_Loiter_hr'],
    marker='o',
    color=color_loiter,
    lw=2.2,
    label='Sustained On-Station Loiter [Hours]'
)
ax1.set_xlabel('ICE Rated Engine Power ($P_{ICE}$) [kW]', fontweight='bold', fontsize=11)
ax1.set_ylabel('Sustained Loiter Endurance [Hours]', color=color_loiter, fontweight='bold', fontsize=11)
ax1.tick_params(axis='y', labelcolor=color_loiter)
ax1.grid(True, linestyle=':', alpha=0.5)

# Secondary Right Axis: Takeoff Ground Roll Distance
ax2 = ax1.twinx()
line2 = ax2.plot(
    df_res['P_ICE_kW'],
    df_res['Ground_Roll_m'],
    marker='s',
    color=color_roll,
    lw=2.2,
    linestyle='--',
    label='Takeoff Ground Roll ($s_{roll}$) [Meters]'
)
ax2.set_ylabel('Ground Roll Distance [m]', color=color_roll, fontweight='bold', fontsize=11)
ax2.tick_params(axis='y', labelcolor=color_roll)

# Runway constraint line (600 m limit)
MAX_ROLL_LIMIT = ms.max_permissible_sroll
line_limit = ax2.axhline(MAX_ROLL_LIMIT, color='#E31A1C', linestyle=':', lw=1.8, label=f'Max Runway Limit ({MAX_ROLL_LIMIT:.0f} m)')

# Upper secondary X-axis: Usable Fuel Capacity [kg]
p_ice_vals = df_res['P_ICE_kW'].values
fuel_vals = df_res['M_Fuel_kg'].values

def pice_to_fuel(x):
    return np.interp(x, p_ice_vals, fuel_vals)

def fuel_to_pice(x):
    return np.interp(x, fuel_vals, p_ice_vals)

ax_fuel = ax1.secondary_xaxis('top', functions=(pice_to_fuel, fuel_to_pice))
ax_fuel.set_xlabel('Available Usable Fuel Mass ($M_{fuel}$) [kg]', fontweight='bold', fontsize=11, labelpad=8)
ax_fuel.tick_params(axis='x')

# Mark the nominal 105 kW baseline with a vertical line
ax1.axvline(105.0, color='gray', linestyle='-.', lw=1.2, alpha=0.8, label='Nominal Baseline (105 kW / 258.6 kg Fuel)')

# Highlight optimum endurance point
opt_row = df_res.loc[df_res['Sustained_Loiter_hr'].idxmax()]
ax1.scatter([opt_row['P_ICE_kW']], [opt_row['Sustained_Loiter_hr']], color='#33A02C', s=120, zorder=5, label=f"Max Loiter: {opt_row['Sustained_Loiter_hr']:.1f} hr @ {opt_row['P_ICE_kW']:.0f} kW")

# Combine legends
handles1, labels1 = ax1.get_legend_handles_labels()
handles2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(handles1 + handles2, labels1 + labels2, loc='upper right', framealpha=0.92, fontsize=8.5)

plt.title('Project Jatayu: Engine Sizing vs. Fuel Mass Sizing Envelope (MTOW = 1000 kg)', fontweight='bold', fontsize=12, pad=38)

plt.figtext(
    0.5, 0.015,
    f"Fixed Parameters: Airframe + Payload = {m_fixed_empty:.1f} kg | Cruise Alt = {ms.cruise_altitude:.0f} m | "
    f"Single Gearbox | Heywood Scaling Active",
    ha='center', fontsize=9.2, style='italic',
    bbox=dict(boxstyle='square,pad=0.4', facecolor='#F0F0F0', edgecolor='#CCCCCC', lw=0.8)
)

fig.tight_layout(rect=[0.0, 0.05, 1.0, 0.95])
plt.savefig('plot_ice_fuel_tradeoff.png', dpi=300)
plt.close(fig)
print("--> Sensitivity plot saved to plot_ice_fuel_tradeoff.png")