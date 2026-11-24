# Pure CI ICE MALE UAV Mission Simulation

Time-marching 11-phase point-mass flight dynamics and powertrain simulation for a **1,000 kg tactical MALE UAV** powered by a standalone **105.0 kW Compression-Ignition (CI) heavy-fuel engine**.

## Key Specifications
* **Airframe:** 1,000.0 kg MTOW | 18.0 m wingspan ($AR = 14.73$) | $22.0\text{ m}^2$ wing area | $C_{D0} = 0.0220$ | $(L/D)_{\max} = 19.13$
* **Powerplant:** 105.0 kW CI engine (182.43 kg installed) + single reduction gearbox (8.00 kg, $\eta_{\text{gb}} = 0.97$) + PTO alternator ($\eta_{\text{alt}} = 0.82$)
* **Energy Storage:** 258.64 kg usable Jet-A1 (243.12 kg mission + 15.52 kg reserve) in 12.93 kg wing bladders (4.8% structural tare)
* **Mission Profile:** 11 segments — Takeoff roll, 15 m obstacle pull-up, initial climb (1,500 m), cruise-climb (3,000 m), 150 km outbound cruise, sustained 20°-bank loiter, 100 km inbound cruise, idle glide descent, approach deceleration, $3^\circ$ glideslope, and flare/braking rollout.

## Quick Start

### 1. Install Dependencies
Requires Python 3.8+ with `ambiance` (ICAO 1993 Standard Atmosphere) and `matplotlib`:
```bash
pip install ambiance numpy matplotlib
```

### 2. Run the Baseline Simulation
Execute your main mission runner script from the repository root:
```bash
python main.py
```
*(Replace `main.py` with your top-level entry script filename if different.)*

### 3. Plug-and-Play Custom Inputs
To test custom airframes, engine ratings, or mission profiles, edit:
* **`jatayu_uav.py`**: MTOW, wing geometry ($b, S$), drag polar ($C_{D0}, K$), propeller diameter, rated ICE power, altitude derating, and BSFC map.
* **`mission_settings.py`**: Cruise/loiter altitudes, outbound/inbound transit distances, runway limits ($s_{\text{roll}} \le 600\text{ m}$), avionics hotel load (0.35 kW), and reserve fuel fraction.

## Baseline Benchmark Results
| Metric | Simulated Value |
| :--- | :--- |
| Takeoff Ground Roll / 15 m Obstacle | 202.98 m / 336.92 m |
| Total Time-to-Climb (0 to 3,000 m) | 10.66 min |
| On-Station Loiter Endurance | **40.37 h** |
| Total Mission Endurance | **43.09 h** |
| Mission Fuel Consumed | 241.67 kg Jet-A1 (Mean BSFC: 247.3 g/kWh) |
| Touchdown Gross Mass / Landing Roll | 758.34 kg / 89.91 m |
