"""
================================================================================
PROJECT JATAYU: TACTICAL MALE UAV (PURE ICE VARIANT)
PHASE 11: TOUCHDOWN, FREE-ROLL, & MECHANICAL BRAKING ROLLOUT
================================================================================
Runway deceleration from touchdown to complete stop under flight idle with the 
105 kW Compression-Ignition engine. Evaluates final mission fuel closure and 
touchdown ground roll distance.
================================================================================
"""

from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.flare as fl

# ------------------------------------------------------------------------------
# 1. INITIALIZE FROM FLARE END-OF-PHASE STATE
# ------------------------------------------------------------------------------
m_initial = float(fl.m)
m = m_initial
V = float(getattr(fl, 'final_velocity', fl.V))
h = float(getattr(fl, 'final_altitude', fl.h))
rho = float(Atmosphere(h).density[0])

t_step = 0.1  # seconds

# Flight idle power setting (6% of sea level rated power)
percent_idle_power = 6.0
P_ICE_Idle = (percent_idle_power / 100.0) * uav.P_ICE_rated  # 6.30 kW for 105 kW block

# Continuous electrical parasitics drawn via engine-driven alternator
P_ICE_avionics = ms.power_avionics / uav.eta_alternator       # kW (~0.427 kW)
P_ICE_shaft = P_ICE_Idle - P_ICE_avionics                     # Net idle brake output [kW]
P_prop_shaft = max(0.0, P_ICE_shaft * uav.eta_gearbox)

eta_prop = 0.0  # Zero thrust during rollout (flat pitch / ground idle)
P_avail = P_prop_shaft * eta_prop

if P_prop_shaft < 0.0:
    print('''
WARNING: Idling power percentage is too low to power avionics alternator! Increase percent_idle_power.
''')

# Dynamic engine load and BSFC at runway elevation
P_ICE_avail = uav.get_ice_available_power(h)
xi_ICE_idle = min(1.0, P_ICE_Idle / P_ICE_avail) if P_ICE_avail > 0.0 else 0.05
bsfc_idle = uav.get_ice_bsfc(P_ICE_Idle, P_ICE_avail)  # g/kWh
m_dot_fuel_gps = (P_ICE_Idle * bsfc_idle) / 3600.0     # g/s

Cd_0_land = float(fl.Cd0_land)
delta_Cl_flaps = 0.40
Cl = uav.Cl_ground + delta_Cl_flaps
Cd = Cd_0_land + ms.K_grd * (Cl ** 2)
mu_r = 0.40  # Braking friction coefficient on dry runway

# ------------------------------------------------------------------------------
# 2. FREE-ROLL PHASE (N_freeroll seconds prior to mechanical brake application)
# ------------------------------------------------------------------------------
N_freeroll = 1.0  # seconds
s_land_roll = N_freeroll * V

m_fuel_freeroll = (m_dot_fuel_gps * N_freeroll) / 1000.0  # kg
m -= m_fuel_freeroll

t = N_freeroll
Phase_Success = True
t_baseline = float(getattr(fl, 'cumulative_time', fl.t_baseline + fl.t_total_flare))

# Structural limit check
if m > uav.max_landing_mass:
    print(
        f"Structural Limit Exceeded: Landing weight {m:.2f} kg exceeds Maximum Landing Weight ({uav.max_landing_mass} kg)"
    )
    Phase_Success = False

# Telemetry log at free-roll completion (Pure ICE signature)
fuel_remaining = m - ms.m_zfw
D_free = 0.5 * rho * (V ** 2) * uav.S * Cd
P_req_free = (D_free * V) / 1000.0
ms.log_telemetry(
    t_baseline + t, 'Landing_Freeroll', h, V, P_req_free,
    P_ICE_Idle, fuel_remaining, xi_ICE_idle, bsfc_idle, m_dot_fuel_gps
)

# ------------------------------------------------------------------------------
# 3. BRAKING ROLLOUT TO FULL STOP
# ------------------------------------------------------------------------------
while V > 0.0 and Phase_Success:
    D = 0.5 * rho * (V ** 2) * uav.S * Cd
    L = 0.5 * rho * (V ** 2) * uav.S * Cl
    net_F = -(D + mu_r * max(0.0, (m * 9.81) - L))  # Decelerating ground force
    a_net = net_F / m

    # Exact fractional time step to full stop
    if V + a_net * t_step <= 0.0:
        dt_stop = -V / a_net
        s_land_roll += 0.5 * (V ** 2) / abs(a_net)
        t += dt_stop
        m -= (m_dot_fuel_gps * dt_stop) / 1000.0
        V = 0.0

        fuel_remaining = m - ms.m_zfw
        cumulative_time = t_baseline + t
        ms.log_telemetry(
            cumulative_time, 'Mission_Complete', h, 0.0, 0.0,
            P_ICE_Idle, fuel_remaining, xi_ICE_idle, bsfc_idle, m_dot_fuel_gps
        )
        break

    s_land_roll += V * t_step
    V += a_net * t_step
    t += t_step

    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0
    m -= m_fuel_step_kg

    fuel_remaining = m - ms.m_zfw
    P_req = (D * V) / 1000.0  # kW
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Landing_Rollout', h, V, P_req,
        P_ICE_Idle, fuel_remaining, xi_ICE_idle, bsfc_idle, m_dot_fuel_gps
    )

    if m < ms.m_zfw:
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    if s_land_roll > ms.max_permissible_sroll:
        print("Touchdown failed: maximum permissible ground roll exceeded.")
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 4. FINAL MISSION DATA EXPORT
# ------------------------------------------------------------------------------
m_fuel_burned = m_initial - m
total_mission_fuel_burned = uav.MTOW - m
total_mission_time = t_baseline + t

if Phase_Success:
    print(f'''
    ------- Touchdown Successful (Pure ICE 105 kW) -------
    ICE Flight Idle: {P_ICE_Idle:.2f} kW | Load Ratio (xi): {xi_ICE_idle:.2f} | BSFC: {bsfc_idle:.1f} g/kWh
    Total Ground Roll Distance: {s_land_roll:.2f} m (Limit: {ms.max_permissible_sroll:.0f} m)
    Time to Full Stop: {t:.2f} seconds
    Fuel Burned During Rollout: {m_fuel_burned*1000.0:.2f} g
    Unused Usable Fuel: {fuel_remaining - ms.mass_reserve:.2f} kg
    Untouched Safety Reserve Fuel: {ms.mass_reserve:.2f} kg
    ======================================================================
    TOTAL MISSION TIME: {total_mission_time/3600.0:.2f} hours ({(total_mission_time/60.0):.1f} mins)
    TOTAL FUEL CONSUMED: {total_mission_fuel_burned:.2f} kg / {ms.mass_fuel:.2f} kg
    ======================================================================\n''')
else:
    print(f'''
    ------- Touchdown Failed -------
    ICE Flight Idle: {P_ICE_Idle:.2f} kW
    Ground Roll Distance: {s_land_roll:.2f} m
    Final Velocity: {V:.2f} m/s | Time: {t:.2f} s
    Final Mass: {m:.2f} kg\n''')