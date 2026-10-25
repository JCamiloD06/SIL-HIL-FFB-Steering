# carga_hil.py
# Torque de carga de la condición HIL-L (Q2-D-034: el force feedback es obligatorio).
#
# El torque se aplica como feedforward en la misma placa (odrive_plant.set_position(θ, torque)),
# sin tocar el lazo de posición ni el controlador lateral. Lo evalúa el hilo espejo de
# SalidaODrive cada ~5 ms con el último estado de AC que leyó registro_ffb_ac.py (~100 Hz).
#
# Convención: τ > 0 empuja el volante a la derecha (θ > 0). Una carga alineante se opone al giro.
# En el paper 1, steer_norm > 0 va con acc_g_x < 0 (r = −0,77 a −0,81 en las 9 corridas de la
# sesión 01, v > 30 km/h; verificado 2026-10-08). Con signo_vjoy = +1, θ > 0 ⇒ a_y = 9,81·acc_g_x < 0,
# así que τ = +k·signo_vjoy·a_y se opone a θ.
#
# Fuentes:
#   modelo  (vía de respaldo, §12.4 del plan): momento autoalineante reducido, lineal en a_y.
#           En régimen permanente F_yf ≈ m·(b/L)·a_y y M_z = t_p·F_yf; en el volante,
#           τ = M_z / i_s = [m·b·t_p / (L·i_s)]·a_y. k (N·m por m/s²) agrupa esos parámetros
#           y se fija antes de la campaña. Limitación declarada: no reproduce la caída del
#           momento alineante cerca del límite de adherencia ni el amortiguamiento.
#   finalff (vía principal si s17_ffb_ac.m la valida): τ = k·signo_ff·finalFF, con k en N·m por
#           unidad de finalFF y signo_ff el que imprime s17 (el que hace que τ se oponga al giro).
#
# Seguridad y "activar sin tocar el controlador": filtro de primer orden (fc_hz), saturación
# ±tau_max_nm, rampa de entrada de rampa_s segundos y caída a cero si el estado de AC tiene más
# de max_edad_s (AC en pausa o memoria sin actualizar).

import math


class CargaHIL:
    def __init__(self, fuente, k, tau_max_nm, signo_vjoy=1, signo_ff=None, fc_hz=10.0,
                 rampa_s=1.0, max_edad_s=0.1):
        if fuente not in ("modelo", "finalff"):
            raise ValueError("fuente debe ser 'modelo' o 'finalff'")
        if fuente == "finalff":
            if signo_ff is None:
                signo_ff = -1
            elif signo_ff not in (1, -1):
                raise ValueError("con fuente finalff hay que dar signo_ff (+1 o −1), el que imprime s17_ffb_ac.m")
        if not (0 < tau_max_nm <= 2.0):
            raise ValueError("tau_max_nm debe estar entre 0 y 2 N·m (tope de seguridad del banco)")
        self.fuente = fuente
        self.k = float(k)
        self.tau_max = float(tau_max_nm)
        self.signo_vjoy = int(signo_vjoy)
        self.signo_ff = signo_ff
        self.fc = float(fc_hz)
        self.rampa_s = float(rampa_s)
        self.max_edad_s = float(max_edad_s)
        self._tau = 0.0
        self._t_prev = None
        self._t_inicio = None
        self.config = {"fuente": fuente, "k": self.k, "tau_max_nm": self.tau_max, "signo_vjoy": self.signo_vjoy,
                       "signo_ff": signo_ff, "fc_hz": self.fc, "rampa_s": self.rampa_s,
                       "max_edad_s": self.max_edad_s,
                       "unidad_k": "N·m por m/s²" if fuente == "modelo" else "N·m por unidad de finalFF"}

    def objetivo(self, estado):
        """Torque sin filtrar ni saturar a partir de una fila de registro_ffb_ac."""
        _, _, final_ff, _, _, acc_g_x, _, _ = estado
        if self.fuente == "modelo":
            return self.k * self.signo_vjoy * 9.81 * acc_g_x
        return self.k * self.signo_ff * final_ff

    def paso(self, t, estado):
        """Torque a aplicar (N·m). estado = última fila de registro_ffb_ac o None."""
        if estado is None or t - estado[0] > self.max_edad_s or not math.isfinite(estado[2] if self.fuente == "finalff" else estado[5]):
            bruto = 0.0
        else:
            if self._t_inicio is None:
                self._t_inicio = t
            bruto = self.objetivo(estado)
        escala = 0.0 if self._t_inicio is None else min(1.0, (t - self._t_inicio) / self.rampa_s)
        bruto = max(-self.tau_max, min(self.tau_max, escala * bruto))
        if self._t_prev is None:
            self._tau = 0.0
        else:
            dt = max(0.0, t - self._t_prev)
            a = 1.0 - math.exp(-2.0 * math.pi * self.fc * dt)
            self._tau += a * (bruto - self._tau)
        self._t_prev = t
        return self._tau


def agregar_argumentos(ap):
    g = ap.add_argument_group("carga HIL-L (Q2-D-034)")
    g.add_argument("--carga-fuente", choices=["modelo", "finalff"], help="fuente del torque de carga (HIL-L)")
    g.add_argument("--carga-k", type=float, help="ganancia: N·m por m/s² (modelo) o N·m por unidad de finalFF")
    g.add_argument("--carga-tau-max", type=float, default=1.0, help="saturación del torque de carga (N·m, por omisión 1.0, máx. 2)")
    g.add_argument("--carga-signo-ff", type=int, choices=[1, -1], default=-1, help="signo de finalFF que imprime s17 (fuente finalff; por omisión -1)")
    g.add_argument("--carga-fc-hz", type=float, default=10.0, help="corte del filtro de primer orden del torque (Hz)")
    g.add_argument("--carga-rampa-s", type=float, default=1.0, help="rampa de entrada del torque (s)")


def desde_argumentos(a, signo_vjoy=1):
    if a.carga_fuente is None:
        return None
    if a.carga_k is None:
        raise SystemExit("HIL-L necesita --carga-k (se fija antes de la campaña; ver carga_hil.py)")
    signo_ff = getattr(a, "carga_signo_ff", None)
    if a.carga_fuente == "finalff" and (signo_ff is None or signo_ff not in (1, -1)):
        signo_ff = -1
    tau_max = getattr(a, "carga_tau_max", None)
    if tau_max is None:
        tau_max = 1.0
    return CargaHIL(a.carga_fuente, a.carga_k, tau_max, signo_vjoy=signo_vjoy,
                    signo_ff=signo_ff, fc_hz=getattr(a, "carga_fc_hz", 10.0) or 10.0,
                    rampa_s=getattr(a, "carga_rampa_s", 1.0) or 1.0)
