# -*- coding: utf-8 -*-
"""
capa_actuador.py
Steering actuator hardware/simulation abstraction layer.

Author: Juan Camilo Diaz-Lopez
Department of Mechanical and Mechatronics Engineering
Universidad Nacional de Colombia, Sede La Paz
Research Paper: Physical Steering Dynamics and Force-Feedback Loading
                in Autonomous Path Tracking
"""

# capa_actuador.py
# Dinámica controlada que se agrega en el PC a la consigna del volante, para el barrido
#
# Orden: retardo puro → filtro de ancho de banda → límite de tasa.
#   retardo   la salida es la entrada de hace retardo_s (retén de orden cero)
#   filtro    PT1 (orden 1) o dos PT1 iguales en cascada (orden 2, amortiguamiento
#             crítico). bw_hz es la frecuencia de −3 dB del filtro completo.
#   tasa      la salida se mueve como mucho tasa_dps·dt por paso
# Se evalúa con el reloj real en cada paso (dt variable): el PT1 usa la discretización
# exacta para una entrada constante en el paso, a = 1 − exp(−ω·dt).
#
# configuración de esta capa se identifica con c03/s13 junto con el servo, y la G_a que entra
# en Γ_E es la del conjunto capa + servo.
#
# Ángulos en radianes, como en SalidaODrive.

import math
from collections import deque


class CapaActuador:
    def __init__(self, bw_hz=None, orden=1, retardo_s=0.0, tasa_dps=None):
        if bw_hz is not None and bw_hz <= 0:
            raise ValueError("bw_hz debe ser positivo")
        if orden not in (1, 2):
            raise ValueError("orden debe ser 1 o 2")
        if retardo_s < 0:
            raise ValueError("retardo_s no puede ser negativo")
        if tasa_dps is not None and tasa_dps <= 0:
            raise ValueError("tasa_dps debe ser positiva")
        self.bw_hz, self.orden, self.retardo_s, self.tasa_dps = bw_hz, orden, float(retardo_s), tasa_dps
        # Dos PT1 iguales tienen su −3 dB conjunto en ω_c·sqrt(√2 − 1); se corrige ω_c para
        # que bw_hz sea el −3 dB del filtro completo.
        if bw_hz is None:
            self.w = None
        elif orden == 1:
            self.w = 2 * math.pi * bw_hz
        else:
            self.w = 2 * math.pi * bw_hz / math.sqrt(math.sqrt(2.0) - 1.0)
        self.tasa = None if tasa_dps is None else math.radians(tasa_dps)
        self._hist = deque()
        self._y = None          # estados del filtro
        self._salida = None
        self._t = None

    @property
    def activa(self):
        return self.w is not None or self.retardo_s > 0 or self.tasa is not None

    @property
    def config(self):
        return {"bw_hz": self.bw_hz, "orden": self.orden, "retardo_s": self.retardo_s,
                "tasa_dps": self.tasa_dps, "activa": self.activa}

    def reiniciar(self, valor=0.0, t=None):
        self._hist.clear()
        self._y = [valor, valor]
        self._salida = valor
        self._salida_inicial = valor
        self._t = t

    def _retrasada(self, t, x):
        self._hist.append((t, x))
        if self.retardo_s <= 0:
            return x
        # Se conserva la última muestra anterior a t − retardo y todas las posteriores.
        while len(self._hist) >= 2 and self._hist[1][0] <= t - self.retardo_s:
            self._hist.popleft()
        t0, x0 = self._hist[0]
        return x0 if t0 <= t - self.retardo_s else self._salida_inicial

    def paso(self, t, x):
    capa = CapaActuador(bw_hz=a.capa_bw_hz, orden=a.capa_orden, retardo_s=a.capa_retardo_ms / 1000.0,
                        tasa_dps=a.capa_tasa_dps)
    return capa if capa.activa else None


def agregar_argumentos(ap):
    g = ap.add_argument_group("capa de dinámica en el PC (barrido,")
    g.add_argument("--capa-bw-hz", type=float, default=None, help="−3 dB del filtro agregado (Hz)")
    g.add_argument("--capa-orden", type=int, default=1, choices=[1, 2], help="1 = PT1, 2 = dos PT1 en cascada")
    g.add_argument("--capa-retardo-ms", type=float, default=0.0, help="retardo puro agregado (ms)")
    g.add_argument("--capa-tasa-dps", type=float, default=None, help="límite de tasa agregado (°/s de volante)")
    return g
