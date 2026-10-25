# Computational Simulation and Path-Tracking Models for Physical Steering Dynamics

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2022b%20or%20newer-orange.svg)](https://www.mathworks.com/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-green.svg)](https://www.python.org/)

This repository contains the computational simulation scripts, vehicle dynamics models, and control algorithms associated with the research paper:

> **Physical Steering Dynamics and Force-Feedback Loading in Autonomous Path Tracking: When Do Software-in-the-Loop Results Transfer to Hardware-in-the-Loop?**  
> *Juan Camilo Díaz-López, Jesús Alberto Lastra-Robles, and Francisco Javier Burgos-Flórez*  
> Department of Mechanical and Mechatronics Engineering, Universidad Nacional de Colombia, Sede La Paz, Colombia.  

---

## Overview

The codebase provides open implementations for simulating, evaluating, and identifying lateral vehicle path-tracking controllers (Pure Pursuit, Stanley, Kinematic MPC, and Actuator-Aware MPC) subjected to physical steering actuator dynamics (transport delay $T_d = 6.38\text{ ms}$, bandwidth $f_{\text{bw}} = 2.78\text{ Hz}$, rate limits $\dot{\delta}_{\max} = 28.74^\circ/\text{s}$, and reactive Pacejka tire self-aligning moments).

---

## Repository Structure

```text
SIL-HIL-FFB-Steering/
├── matlab/                                 # MATLAB simulation and analytical modeling
│   ├── simulacion_lazo_cerrado.m           # Closed-loop lateral simulation of vehicle and steering dynamics
│   ├── simulacion_lazo_identificado.m      # Closed-loop simulation with identified actuator dynamics
│   ├── identificacion_actuador.m           # Transfer function identification of steering actuator Ga(s)
│   ├── modelo_vehiculo_2dof.m              # 2-DOF lateral bicycle dynamics and understeer calculation
│   ├── identificar_vehiculo.m              # Linear bicycle model fitting and validation
│   ├── espectro_comando.m                  # Power spectral density calculation of steering commands
│   ├── indice_compatibilidad_gamma.m       # Computation of Spectral Compatibility Index (Gamma_E)
│   ├── evaluacion_resultados_benchmark.m   # Lateral tracking RMSE and completion metrics evaluation
│   └── simulacion_transitoria_chicane.m    # High-speed transient chicane dynamic analysis
├── python/                                 # Real-time control and Hardware-in-the-Loop algorithms
│   ├── mpc_actuator_aware.py               # Actuator-Aware MPC QP formulation with rate bounds
│   ├── carga_hil.py                        # Dynamic tire self-aligning torque feedforward coupling
│   ├── capa_actuador.py                    # Actuator simulation and hardware abstraction layer
│   ├── salida_odrive.py                    # Direct-drive BLDC communication & 200 Hz position ramp
│   └── correr_trialsteer.py                # Real-time simulation and benchmark orchestrator
├── .gitignore                              # Git exclusion rules
└── README.md                               # Repository documentation
```

---

## Requirements

- **MATLAB:** R2022b or later (Toolboxes: *Control System Toolbox*, *Signal Processing Toolbox*, *Statistics and Machine Learning Toolbox*).
- **Python:** 3.10 or later (Libraries: `numpy`, `scipy`, `odrive`, `pyvjoy`).

---

## Usage

### 1. Closed-Loop Lateral Tracking Simulation (MATLAB)
To simulate the vehicle and steering actuator in closed loop and evaluate the onset of delay-induced limit cycles:
```matlab
cd matlab
simulacion_lazo_cerrado
```

### 2. Steering Actuator Transfer Function Identification
To identify the second-order model with pure delay ($K = 1.044, \omega_n = 15.93\text{ rad/s}, \zeta = 0.264, T_d = 6.38\text{ ms}$):
```matlab
cd matlab
identificacion_actuador
```

### 3. Spectral Compatibility Index ($\Gamma_E$)
To compute the command high-frequency energy ratio and predict hardware tracking degradation:
```matlab
cd matlab
indice_compatibilidad_gamma
```

### 4. Actuator-Aware Model Predictive Control (Python)
To execute the discrete-time rate-constrained QP optimization:
```bash
cd python
python mpc_actuator_aware.py
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
