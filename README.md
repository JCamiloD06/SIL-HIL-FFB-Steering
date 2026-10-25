# Computational Simulation and Path-Tracking Models for Physical Steering Dynamics

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2022b%20or%20newer-orange.svg)](https://www.mathworks.com/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-green.svg)](https://www.python.org/)

This repository contains the computational simulation scripts, vehicle dynamics models, and control algorithms associated with the research paper:

> **Physical Steering Dynamics and Force-Feedback Loading in Autonomous Path Tracking: When Do Software-in-the-Loop Results Transfer to Hardware-in-the-Loop?**  
> *Juan Camilo Díaz-López, Jesús Alberto Lastra-Robles, and Francisco Javier Burgos-Flórez*  
> Department of Mechanical and Mechatronics Engineering, Universidad Nacional de Colombia, Sede La Paz, Colombia.  
> Submitted to: *Ingeniería e Investigación*.

---

## Overview

The codebase provides open implementations for simulating, evaluating, and identifying lateral vehicle path-tracking controllers (Pure Pursuit, Stanley, Kinematic MPC, and Actuator-Aware MPC) subjected to physical steering actuator dynamics:
- Transport delay: $T_d = 6.38\text{ ms}$
- Actuator bandwidth: $f_{\text{bw}} = 2.78\text{ Hz}$ (second-order $\omega_n = 15.93\text{ rad/s}$, $\zeta = 0.264$)
- Front road-wheel slew-rate limit: $\dot{\delta}_{\max} = 28.74^\circ/\text{s}$ (derived from the physical $560.0^\circ/\text{s}$ direct-drive servomotor limit with steering column ratio $k_w = 19.49\text{ deg}_{\text{hw}}/\text{deg}_{\text{rw}}$)
- Discrete rate constraint: $|\Delta u| \le 1.437^\circ$ ($0.0251\text{ rad}$) per $50\text{ ms}$ control cycle
- Reactive tire self-aligning torque loading ($M_z$) via force-feedback coupling.

> **Note on Standalone Reproducibility:**  
> The numerical simulation scripts in the `matlab/` directory and `python/mpc_actuator_aware.py` are **completely standalone** and execute without external hardware or proprietary simulator dependencies. The real-time Hardware-in-the-Loop scripts (`correr_trialsteer.py`, `salida_odrive.py`, and `carga_hil.py`) provide the exact open-source motor driver and telemetry interface used with the physical ODrive steering test bench and Assetto Corsa (v1.16.4). Multi-gigabyte raw experimental telemetry from the 371 track runs is available upon reasonable academic request from the corresponding author (`jcdiazlo@unal.edu.co`).

---

## Repository Structure

```text
SIL-HIL-FFB-Steering/
|-- matlab/                                 # MATLAB analytical and simulation models (Standalone)
|   |-- simulacion_lazo_cerrado.m           # Closed-loop lateral simulation and limit-cycle analysis
|   |-- simulacion_lazo_identificado.m      # Closed-loop simulation with identified actuator Ga(s)
|   |-- identificacion_actuador.m           # Second-order + delay transfer function identification
|   |-- modelo_vehiculo_2dof.m              # 2-DOF lateral bicycle dynamics and understeer calculation
|   |-- identificar_vehiculo.m              # Linear bicycle model parameter fitting and validation
|   |-- espectro_comando.m                  # Power spectral density calculation of steering commands
|   |-- indice_compatibilidad_gamma.m       # Computation of Spectral Compatibility Index (Gamma_E)
|   |-- evaluacion_resultados_benchmark.m   # Lateral tracking RMSE and completion metrics evaluation
|   |-- simulacion_transitoria_chicane.m    # High-speed transient chicane dynamic analysis
|   |-- s10_caracterizacion_escalones.m     # Experimental step response actuator characterization
|   |-- s13_identificacion_banco.m          # Actuator frequency response identification
|   |-- s14_espectro_comando.m              # Controller command frequency spectrum processing
|   |-- s15_cadena_vehicular.m              # End-to-end steering chain validation
|   |-- s17_ffb_ac.m                        # Assetto Corsa force-feedback telemetric coupling
|   |-- s21_servo_volante.m                 # Closed-loop steering servomotor response
|   |-- s24_identificar_vehiculo.m          # Experimental vehicle dynamics identification
|   |-- s29_indice_compatibilidad_gamma.m   # Spectral compatibility index evaluation
|   |-- s31_analisis_estadistico_vsd.m      # Factorial experimental campaign statistical analysis
|   \-- s37_analisis_campana_hill.m         # Complete HIL-L vs HIL-U experimental comparison
|-- python/                                 # Real-time control and Hardware-in-the-Loop algorithms
|   |-- mpc_actuator_aware.py               # Actuator-Aware MPC QP optimization (Standalone executable)
|   |-- carga_hil.py                        # Dynamic tire self-aligning torque feedforward coupling
|   |-- capa_actuador.py                    # Actuator simulation and hardware abstraction layer
|   |-- salida_odrive.py                    # Direct-drive BLDC communication & 200 Hz position ramp
|   \-- correr_trialsteer.py                # Real-time HIL benchmark orchestrator
|-- .gitignore                              # Git exclusion rules
|-- LICENSE                                 # MIT License
\-- README.md                               # Repository documentation
```

---

## Requirements

- **MATLAB:** R2022b or later (Toolboxes: *Control System Toolbox*, *Signal Processing Toolbox*, *Statistics and Machine Learning Toolbox*).
- **Python:** 3.10 or later (Libraries: `numpy`, `scipy`, `osqp`). Optional for HIL hardware: `odrive`, `pyvjoy`.

---

## Quick Start & Usage

### 1. Standalone Actuator-Aware MPC Optimization (Python)
To execute the discrete-time rate-constrained QP optimization with OSQP (reproduces the paper QP formulation, rate limits, and execution timing):
```bash
cd python
python mpc_actuator_aware.py
```

### 2. Closed-Loop Lateral Tracking Simulation (MATLAB)
To simulate the vehicle and steering actuator in closed loop and evaluate delay-induced limit cycles:
```matlab
cd matlab
simulacion_lazo_cerrado
```

### 3. Steering Actuator Transfer Function Identification
To identify the second-order model with pure transport delay ($K = 1.044, \omega_n = 15.93\text{ rad/s}, \zeta = 0.264, T_d = 6.38\text{ ms}$):
```matlab
cd matlab
identificacion_actuador
```

### 4. Spectral Compatibility Index ($\Gamma_E$)
To compute the command high-frequency energy ratio and screen for actuator phase-lag vulnerability:
```matlab
cd matlab
indice_compatibilidad_gamma
```

---

## Author & Contact

**Juan Camilo Díaz-López**  
Department of Mechanical and Mechatronics Engineering  
Universidad Nacional de Colombia, Sede La Paz, Colombia  
Email: `jcdiazlo@unal.edu.co`

---

## License

This project is licensed under the MIT License.
