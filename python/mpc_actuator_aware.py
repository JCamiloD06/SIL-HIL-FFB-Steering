# -*- coding: utf-8 -*-
"""
mpc_actuator_aware.py
Actuator-Aware MPC QP formulation with slew-rate constraints.

Author: Juan Camilo Diaz-Lopez
Department of Mechanical and Mechatronics Engineering
Universidad Nacional de Colombia, Sede La Paz
Research Paper: Physical Steering Dynamics and Force-Feedback Loading
                in Autonomous Path Tracking
"""

import math
import numpy as np
import osqp
import scipy.linalg as la
import scipy.sparse as sp
import sys
from pathlib import Path

# Fallback definitions for standalone execution without external dependencies
try:
    from controladores.base import ControladorLateral, SalidaControlador, EntradaLateral
except ImportError:
    from dataclasses import dataclass

    class MockTrazada:
        """Synthetic track representation for standalone simulation."""
        def curvatura_adelante(self, idx, s):
            return 0.0

    @dataclass
    class EntradaLateral:
        trazada: object = MockTrazada()
        v_ms: float = 38.89
        L: float = 2.62
        psi: float = 0.0
        x_tras: float = 0.0
        z_tras: float = 0.0
        x_del: float = 0.0
        z_del: float = 0.0
        idx_tras: int = 0
        e_y_tras: float = 0.0
        e_psi_tras: float = 0.0
        idx_del: int = 0
        e_y_del: float = 0.0
        e_psi_del: float = 0.0
        delta_prev: float = 0.0

    @dataclass
    class SalidaControlador:
        delta: float = 0.0
        estado: str = 'ok'
        iteraciones: int = 0
        respaldo: int = 0
        extra1: float = float('nan')
        extra2: float = float('nan')

    class ControladorLateral:
        nombre = 'base'
        def __init__(self, parametros):
            self.parametros = dict(parametros)
        def reiniciar(self):
            pass
        def calcular(self, entrada):
            raise NotImplementedError

class MPCActuatorAware(ControladorLateral):
    nombre = "mpc_actuator_aware"

    def __init__(self, parametros, ts_s, delta_max_rad, tasa_max_rad_s):
        super().__init__(parametros)
        p = self.parametros
        self.N = int(p.get("N", 20))
        if self.N < 2:
            raise ValueError("N debe ser al menos 2")

        self.Qy = float(p.get("Qy", 1.0))
        self.Qpsi = float(p.get("Qpsi", 33.167603))
        self.Rd = float(p.get("Rdelta", 8.147265))
        self.Rdd = float(p.get("Rddelta", 5.0))
        self.parametros["Rddelta"] = self.Rdd
        self.tiempo_max = float(p.get("tiempo_max_s", 0.04))
        self.ts = float(ts_s)
        self.dmax = float(delta_max_rad)

        # Parámetros identificados del actuador físico G_a(s) (Fase 1)
        self.K_act = float(p.get("actuator_K", 1.0440))
        self.fn_act = float(p.get("actuator_fn_hz", 2.5354))
        self.wn_act = 2.0 * np.pi * self.fn_act
        self.zeta_act = float(p.get("actuator_zeta", 0.2641))

        # Límite físico de slew-rate del volante (560 deg/s identificado en banco)
        slew_volante_dps = float(p.get("actuator_slew_wheel_dps", 560.0))
        rango_volante_deg = float(p.get("rango_volante_deg", 900.0))
        dmax_deg = math.degrees(self.dmax)
        self.kw = (rango_volante_deg / 2.0) / dmax_deg  # ~19.49 deg_volante / deg_rueda
        slew_rueda_rad_s = math.radians(slew_volante_dps / self.kw)

        # El paso admisible es el mínimo entre la configuración y el límite físico del motor
        paso_cfg = float(tasa_max_rad_s) * self.ts
        paso_fisico = slew_rueda_rad_s * self.ts
        self.paso = min(paso_cfg, paso_fisico)

        # Matrices continuas del actuador
        self.A_act_c = np.array([
            [0.0, 1.0],
            [-self.wn_act**2, -2.0 * self.zeta_act * self.wn_act]
        ])
        self.B_act_c = np.array([[0.0], [self.K_act * self.wn_act**2]])

        # Discretización exacta ZOH del subsistema del actuador para el observador interno
        M_act = np.zeros((3, 3))
        M_act[:2, :2] = self.A_act_c
        M_act[:2, 2:] = self.B_act_c
        eM_act = la.expm(M_act * self.ts)
        self.A_act_d = eM_act[:2, :2]
        self.B_act_d = eM_act[:2, 2:]

        # Estructura del QP
        N = self.N
        self.D = np.eye(N) - np.eye(N, k=-1)
        self.DtD = self.D.T @ self.D
        self.Qdiag = np.tile([self.Qy, self.Qpsi, 0.0, 0.0], N)
        self.A_rest = sp.csc_matrix(np.vstack([np.eye(N), self.D]))

        # Patrón fijo del triángulo superior de P
        indptr, indices = [0], []
        for j in range(N):
            indices.extend(range(j + 1))
            indptr.append(len(indices))
        self._indices = np.array(indices, dtype=np.int32)
        self._indptr = np.array(indptr, dtype=np.int32)

        self.prob = None
        self.secuencia = np.zeros(N)
        self.fallos_consecutivos = 0

        # Estado interno del actuador: [delta_a, ddelta_a]
        self.x_act = np.zeros(2)
        self.ultimo_u = 0.0
        self.inicializado = False

    def reiniciar(self):
        self.prob = None
        self.secuencia = np.zeros(self.N)
        self.fallos_consecutivos = 0
        self.x_act = np.zeros(2)
        self.ultimo_u = 0.0
        self.inicializado = False

    def _datos_triu(self, H):
        return np.concatenate([H[: j + 1, j] for j in range(self.N)])

    def _prediccion(self, v, L, kappas):
        N, ts = self.N, self.ts

        # Matriz continua 4x4
        Ac = np.zeros((4, 4))
        Ac[0, 1] = v
        Ac[1, 2] = v / L
        Ac[2, 2:] = self.A_act_c[0, :]
        Ac[3, 2:] = self.A_act_c[1, :]

        Bc = np.zeros((4, 1))
        Bc[2:, :] = self.B_act_c

        # Discretización exacta ZOH via Van Loan
        M = np.zeros((5, 5))
        M[:4, :4] = Ac
        M[:4, 4:] = Bc
        eM = la.expm(M * ts)
        Ad = eM[:4, :4]
        Bd = eM[:4, 4:]

        Sx = np.zeros((4 * N, 4))
        Su = np.zeros((4 * N, N))
        c = np.zeros(4 * N)
        Mx = np.eye(4)
        Mu = np.zeros((4, N))
        mc = np.zeros(4)

        for k in range(N):
            Mx = Ad @ Mx
            Mu = Ad @ Mu
            Mu[:, k] += Bd.ravel()

            # Perturbación de curvatura en [0, ts]
            # dy = -0.5 * v^2 * ts^2 * kappa
            # dpsi = -v * ts * kappa
            w_k = np.array([-0.5 * v**2 * ts**2 * kappas[k], -v * ts * kappas[k], 0.0, 0.0])
            mc = Ad @ mc + w_k

            Sx[4 * k : 4 * k + 4] = Mx
            Su[4 * k : 4 * k + 4] = Mu
            c[4 * k : 4 * k + 4] = mc

        return Sx, Su, c

    def matrices_qp(self, e):
        N = self.N
        v = max(e.v_ms, 0.0)
        L = e.L

        # Curvatura adelantada en el horizonte
        kappas = np.array([e.trazada.curvatura_adelante(e.idx_tras, v * self.ts * k)
                           for k in range(N)])

        # Consigna feedforward de régimen estático para el actuador:
        # En régimen delta_a = L * kappa. Como G_a(0) = K_act, u_ff = atan(L * kappa) / K_act
        u_ff = np.arctan(L * kappas) / self.K_act

        Sx, Su, c = self._prediccion(v, L, kappas)

        # Actualización del estado interno del actuador
        if not self.inicializado:
            self.x_act = np.array([e.delta_prev, 0.0])
            self.ultimo_u = e.delta_prev
            self.inicializado = True
        else:
            # Propagar el paso de simulación con el último u enviado
            self.x_act = self.A_act_d @ self.x_act + self.B_act_d.ravel() * self.ultimo_u

        # Vector de estado inicial aumentado x0
        x0 = np.array([e.e_y_tras, e.e_psi_tras, self.x_act[0], self.x_act[1]])

        x_libre = Sx @ x0 + c
        SuQ = Su.T * self.Qdiag
        H = 2.0 * (SuQ @ Su + self.Rd * np.eye(N) + self.Rdd * self.DtD)

        d0 = np.zeros(N)
        d0[0] = self.ultimo_u
        q = 2.0 * (SuQ @ x_libre - self.Rd * u_ff - self.Rdd * (self.D.T @ d0))

        lim = np.full(N, self.dmax)
        l = np.concatenate([-lim, d0 - self.paso])
        u = np.concatenate([lim, d0 + self.paso])
        return H, q, l, u

    def calcular(self, e):
        N = self.N
        H, q, l, u = self.matrices_qp(e)
        Px = self._datos_triu(H)

        if self.prob is None:
            P = sp.csc_matrix((Px, self._indices, self._indptr), shape=(N, N))
            self.prob = osqp.OSQP()
            self.prob.setup(P, q, self.A_rest, l, u, verbose=False, warm_starting=True,
                            polishing=False, time_limit=self.tiempo_max,
                            eps_abs=1e-4, eps_rel=1e-4, max_iter=4000)
        else:
            self.prob.update(Px=Px, q=q, l=l, u=u)

        candidata = np.append(self.secuencia[1:], self.secuencia[-1])
        self.prob.warm_start(x=candidata)
        res = self.prob.solve()
        estado = str(res.info.status)
        x = res.x

        if estado == "solved" and x is not None and np.all(np.isfinite(x)):
            self.secuencia = np.array(x, dtype=float)
            respaldo = 0
            self.fallos_consecutivos = 0
        else:
            self.secuencia = candidata
            respaldo = 1
            self.fallos_consecutivos += 1

        u_cmd = float(self.secuencia[0])
        self.ultimo_u = u_cmd

        return SalidaControlador(
            delta=u_cmd,
            estado=estado,
            iteraciones=int(res.info.iter),
            respaldo=respaldo,
            extra1=float(res.info.run_time) * 1000.0,
            extra2=float(res.info.obj_val)
        )

if __name__ == '__main__':
    print('=' * 70)
    print('Actuator-Aware MPC (AA-MPC) Standalone Verification Test')
    print('=' * 70)
    
    # Paper nominal parameters
    ts_s = 0.05
    delta_max_rad = math.radians(23.09)
    tasa_max_rad_s = math.radians(28.7405)  # 560 deg/s / 19.49 = 28.74 deg/s
    
    params = {
        'N': 20,
        'Qy': 12.0,
        'Qpsi': 8.0,
        'Rdelta': 1.0,
        'Rddelta': 5.0,  # Paper rate-increment weighting R_Delta = 5.0
        'actuator_K': 1.0440,
        'actuator_fn_hz': 2.5354,
        'actuator_zeta': 0.2641,
        'actuator_slew_wheel_dps': 560.0,
        'rango_volante_deg': 900.0,
        'tiempo_max_s': 0.04
    }
    
    mpc = MPCActuatorAware(params, ts_s, delta_max_rad, tasa_max_rad_s)
    
    # Synthetic test scenario: vehicle at 140 km/h with 0.3 m lateral error
    try:
        trazada_obj = MockTrazada()
    except NameError:
        class MockTrazada:
            def curvatura_adelante(self, idx, s):
                return 0.0
        trazada_obj = MockTrazada()

    test_in = EntradaLateral(
        trazada=trazada_obj,
        v_ms=38.89,  # 140 km/h
        L=2.62,
        e_y_tras=0.30,
        e_psi_tras=0.02,
        delta_prev=0.0
    )
    
    salida = mpc.calcular(test_in)
    
    print(f'Prediction horizon: N = {mpc.N} ({mpc.N * ts_s:.2f} s)')
    print(f'Steering ratio kw:  {mpc.kw:.2f} deg_hw / deg_rw')
    print(f'Road-wheel slew-rate bound: {math.degrees(mpc.paso / ts_s):.2f} deg/s')
    print(f'Per-sample bound (|Delta u|): {math.degrees(mpc.paso):.3f} deg ({mpc.paso:.4f} rad)')
    print(f'Rate-penalty R_Delta:        {mpc.Rdd}')
    print(f'Optimization status:         {salida.estado}')
    print(f'Commanded road-wheel delta:  {math.degrees(salida.delta):.3f} deg ({salida.delta:.4f} rad)')
    print(f'Commanded handwheel theta:   {math.degrees(salida.delta) * mpc.kw:.2f} deg')
    print(f'Solver iterations:           {salida.iteraciones}')
    print(f'QP execution time:           {salida.extra1:.2f} ms')
    print('=' * 70)
    print('[SUCCESS] Standalone Actuator-Aware MPC executed without errors.')
    print('=' * 70)
