# salida_odrive.py
# Salida de dirección HIL para TrialSteer v1.0.1 (Q2-D-013, 2026-10-07).
#
# Misma interfaz que plataforma.actuador.SalidaVJoy: enviar(delta_rad, u_long)
# devuelve (gas, freno, norm, sat_eje) y centrar() suelta todo. Los
# controladores de TrialSteer no se tocan y no saben que hay un volante.
#
# Cadena HIL-U:
#   δ de rueda (rad) → θ_cmd = δ·k (volante) → placa Odrive-Wheel (lazo de posición a 8 kHz)
#   → encoder θ_meas → vJoy X = θ_meas/(rango/2) → Assetto Corsa
# con k = (rango/2) / rueda_rad_eje_completo. Con rango 360° y 0,403 rad, k ≈ 7,8;
# con 900° (Q2-D-035: LOCK del volante vJoy en el controls.ini de AC), k ≈ 19,5. El valor por omisión de rango_volante_deg (360) solo
# queda por compatibilidad; los scripts que llaman a esta clase lo exigen explícito.
# Así el eje de vJoy a fondo es el mismo en SIL y en HIL: δ/0,403 en SIL y
# δ_meas/0,403 en HIL.
#
# Un hilo "espejo" lee el encoder cada ~5 ms, actualiza el eje X de vJoy con
# θ_meas y reenvía la última consigna (alimenta el watchdog de la placa). Así
# AC ve el volante físico de forma casi continua, no muestreado a 20 Hz, y la
# placa no se suelta si el juego pausa y el bucle de TrialSteer deja de enviar.
# Si el proceso muere, el hilo muere con él y el watchdog suelta el motor.
#
# Cortes (motor a IDLE y la corrida se detiene en el siguiente ciclo):
#   |θ_meas| > corte_theta_deg, |ω| > corte_omega_dps,
#   |θ_cmd − θ_meas| > corte_error_deg sostenido más de corte_error_s,
#   placa fuera de lazo cerrado o con error.
#
# Capa de dinámica en el PC (opcional, barrido Q2-D-021/Q2-D-025, capa_actuador.py): si se pasa
# capa, la consigna pedida θ_pedido = k·δ pasa por la capa y la placa recibe θ_cmd = capa(θ_pedido),
# evaluada en cada ciclo de TrialSteer y en cada paso del hilo espejo (~5 ms). Sin capa,
# θ_cmd = θ_pedido, como antes.
#
# Registro eléctrico (plan Q1, apéndice A; 2026-10-08): el hilo espejo lee Iq y Vbus cada
# electrico_cada_s (0,1 s) y la temperatura del FET junto con la salud (0,5 s). Son lecturas
# opcionales: si una propiedad no existe en el firmware, se registra NaN y se deja de pedir
# tras 3 fallos seguidos, sin cortar la corrida. La versión del firmware va en config.
#
# Carga HIL-L (Q2-D-034, carga_hil.py): si se pasa carga y estado_ac (función que devuelve la última
# fila de registro_ffb_ac), el hilo espejo calcula el torque en cada paso y lo manda con la consigna
# de posición como feedforward. Sin carga, el comando a la placa es el mismo de HIL-U.
# Con ffb (ffb_hid.FFBHid, opción A, 2026-10-08) el torque va por el force feedback USB HID de
# Odrive-Wheel, la única vía que llega al motor: su pila FFB reescribe input_torque, así que el
# torque por el puerto serie se pierde (diag_input_torque.py).
#
# Latencia (plan §8.4): cada fila del espejo guarda el instante de la lectura del encoder
# (t_leido_s) y el de la actualización de vJoy (t_vjoy_s); T2 = t_vjoy − t_leido.

import math
import threading
import time

AXIS_MAX = 32767
PROP_IQ = "axis0.motor.current_control.Iq_measured"
PROP_VBUS = "vbus_voltage"
# Nombre de la temperatura del FET según la versión de ODrive; se usa el primero que responda.
PROPS_TEMP_FET = ["axis0.motor.fet_thermistor.temperature", "axis0.fet_thermistor.temperature"]
PROPS_FIRMWARE = ["fw_version_major", "fw_version_minor", "fw_version_revision", "fw_version_unreleased"]


class CorteHIL(RuntimeError):
    pass


def valor_eje(norm):
    """Mismo redondeo que SalidaVJoy.enviar de TrialSteer."""
    return max(1, min(32767, int((norm + 1.0) / 2.0 * 32766 + 1)))


class SalidaODrive:
    def __init__(self, rueda_rad_eje_completo, signo_vjoy, planta, rango_volante_deg=360.0,
                 periodo_espejo_s=0.005, corte_theta_deg=None, corte_omega_dps=400.0,
                 corte_error_deg=45.0, corte_error_s=0.5, salud_cada_s=0.5, id_dispositivo=1,
                 vjoy=None, capa=None, electrico_cada_s=0.1, carga=None, estado_ac=None, ffb=None,
                 limite_volante_deg=250.0, arranque_sil_kmh=None, arranque_sil_s=1.0,
                 tasa_arranque_dps=90.0, consigna_interpolada=False,
                 zona_muerta_volante_deg=0.0):
        if vjoy is None:
            import pyvjoy  # solo en vivo
            vjoy = pyvjoy.VJoyDevice(id_dispositivo)
        self.j = vjoy
        self.pl = planta
        self.rueda = float(rueda_rad_eje_completo)
        self.signo = int(signo_vjoy)
        self.medio_rango = math.radians(rango_volante_deg / 2.0)
        self.k = self.medio_rango / self.rueda
        self.periodo = float(periodo_espejo_s)
        # Tope del banco (2026-10-08, primera HIL-U): en la salida desde parado el lazo cerrado llevó
        # el volante a +355° (consigna +320° y sobrepaso) y algo lo retuvo cerca de +350° con 4,5 A
        # (cable o roce). La consigna se satura en ±limite_volante_deg (250°) y el corte por ángulo
        # queda en limite + 80° (330°), antes del obstáculo. En la vuelta medida del paper 1 el
        # volante no pasa de ~155° con 900°: el tope solo actúa en la salida desde parado.
        self.limite = math.radians(limite_volante_deg)
        # Arranque en SIL (2026-10-08, primera HIL-U completa): en la salida desde parado Stanley
        # oscilaba cada vez más con el volante físico (atraso ~0,25 s y sobrepaso) y el carro se salía
        # de la pista; en SIL la misma salida se amortigua en 4 s. Hasta que el carro pasa de
        # arranque_sil_kmh durante arranque_sil_s, AC recibe el comando del controlador (como en SIL)
        # mientras el volante lo sigue; después recibe θ_meas (HIL) y no vuelve atrás. La vuelta
        # medida es toda HIL; el cambio queda registrado (hil_activo por ciclo, ciclo_inicio_hil).
        self.arranque = arranque_sil_kmh
        self.arranque_s = float(arranque_sil_s)
        self.hil_activo = not arranque_sil_kmh
        self._t_sobre = None
        self.ciclo_inicio_hil = 0 if self.hil_activo else None
        self._theta_sin_tope = 0.0
        # Durante el arranque en SIL el volante sigue el comando a tasa limitada (tasa_arranque_dps).
        # Motivo (2026-10-08): las frenadas desde 600–780 °/s de la salida desde parado hacen saltar
        # el engrane del encoder (dos veces hoy). En ese tramo AC no lee el volante, así que no hace
        # falta que lo siga rápido; al activarse HIL los comandos ya son de pocos grados.
        self.tasa_arranque = math.radians(tasa_arranque_dps)
        # Consigna interpolada (2026-10-08, s21_servo_volante.m): en vez de escalones cada ciclo del
        # controlador (20 Hz), el hilo espejo extrapola la consigna con la pendiente de los dos últimos
        # comandos (orden uno, con adelanto de fase) y la manda con esa velocidad como vel_ff.
        # Con la capa del barrido activa no se usa (la capa ya da una consigna continua).
        self.interpolada = bool(consigna_interpolada) and (capa is None or not capa.activa)
        self.zona_muerta_volante = math.radians(float(zona_muerta_volante_deg))
        self._theta_filtrado = 0.0
        self._theta_k = 0.0
        self._v_k = 0.0
        self._t_k = None
        self._t_prev_envio = None
        self.n_limitados = 0
        self.corte_theta = math.radians(corte_theta_deg if corte_theta_deg is not None
                                        else min(1.05 * rango_volante_deg / 2.0, limite_volante_deg + 80.0))
        self.corte_omega = math.radians(corte_omega_dps)
        self.corte_error = math.radians(corte_error_deg)
        self.corte_error_s = float(corte_error_s)
        self.salud_cada_s = float(salud_cada_s)
        self.capa = capa if (capa is not None and capa.activa) else None
        if arranque_sil_kmh and estado_ac is None:
            raise ValueError("el arranque en SIL necesita estado_ac (velocidad de AC)")
        if carga is not None and estado_ac is None:
            raise ValueError("la carga necesita estado_ac (registro de la memoria de AC)")
        self.carga = carga
        self.estado_ac = estado_ac
        self.ffb = ffb
        self.tau = 0.0
        self._t_leido = self._t_vjoy = float("nan")

        self.lock = threading.Lock()
        self.theta_cmd = 0.0
        self.theta_pedido = 0.0
        self.theta_meas = 0.0
        self.omega = 0.0
        self.corte = None
        self.t_error_desde = None
        self._t_igual_desde = None
        self.ciclos = []    # una fila por llamada a enviar (= ciclo de TrialSteer)
        self.espejo = []    # una fila por lectura del hilo
        self.electrico = []  # t, Iq, Vbus, temperatura del FET
        self.electrico_cada_s = float(electrico_cada_s)
        self._fallos = {}
        self._prop_temp = None
        self._temp = float("nan")
        firmware = {p: self._leer_opcional(p) for p in PROPS_FIRMWARE}
        self.config = {
            "rango_volante_deg": rango_volante_deg, "k_volante_por_rueda": self.k,
            "rueda_rad_eje_completo": self.rueda, "signo_vjoy": self.signo,
            "periodo_espejo_s": self.periodo, "corte_theta_deg": math.degrees(self.corte_theta),
            "corte_omega_dps": corte_omega_dps, "corte_error_deg": corte_error_deg,
            "corte_error_s": corte_error_s, "placa": getattr(planta, "config", None),
            "capa": self.capa.config if self.capa is not None else None,
            "firmware": firmware, "electrico_cada_s": self.electrico_cada_s,
            "prop_iq": PROP_IQ, "prop_vbus": PROP_VBUS,
            "limite_volante_deg": limite_volante_deg,
            "arranque_sil_kmh": arranque_sil_kmh, "arranque_sil_s": self.arranque_s,
            "tasa_arranque_dps": tasa_arranque_dps, "consigna_interpolada": self.interpolada,
            "zona_muerta_volante_deg": float(zona_muerta_volante_deg),
            "carga": carga.config if carga is not None else None,
            "ffb": ffb.config if ffb is not None else None,
        }

        self._vivo = True
        self._hilo = threading.Thread(target=self._bucle_espejo, name="espejo_odrive", daemon=True)
        self._hilo.start()

    # --- lecturas eléctricas opcionales; nunca cortan la corrida ---
    def _leer_opcional(self, prop):
        if self._fallos.get(prop, 0) >= 3:
            return float("nan")
        try:
            v = float(self.pl.r(prop))
            self._fallos[prop] = 0
            return v
        except Exception:
            self._fallos[prop] = self._fallos.get(prop, 0) + 1
            return float("nan")

    def _leer_temperatura(self):
        if self._prop_temp is None:
            for p in PROPS_TEMP_FET:
                v = self._leer_opcional(p)
                if v == v:   # no es NaN
                    self._prop_temp = p
                    self.config["prop_temp_fet"] = p
                    return v
            self._prop_temp = ""   # ninguna respondió
            self.config["prop_temp_fet"] = None
            return float("nan")
        return self._leer_opcional(self._prop_temp) if self._prop_temp else float("nan")

    # --- lectura y vJoy, siempre con el lock tomado ---
    def _leer_y_reflejar(self):
        th, w = self.pl.get_state()
        self._t_leido = time.perf_counter()
        self.theta_meas, self.omega = th, w
        # En el arranque en SIL, AC recibe el comando del controlador sin el tope del banco.
        ratio = self.signo * (th if self.hil_activo else self._theta_sin_tope) / self.medio_rango
        norm = max(-1.0, min(1.0, ratio))
        self.j.data.wAxisX = valor_eje(norm)
        self.j.update()
        self._t_vjoy = time.perf_counter()
        return th, w, norm, abs(ratio) > 1.0

    def _revisar(self, t):
        th, w = self.theta_meas, self.omega
        if abs(th) > self.corte_theta:
            return f"|θ_meas| = {math.degrees(th):+.1f}° supera {math.degrees(self.corte_theta):.0f}°"
        if abs(w) > self.corte_omega:
            return f"|ω| = {math.degrees(w):+.0f} °/s supera {math.degrees(self.corte_omega):.0f} °/s"
        # Lectura congelada (c02, 2026-10-08: el ángulo quedó en un valor exacto 2,3 s con el volante
        # quieto y la placa sin error). Un valor idéntico durante 0,5 s con más de 5° de error no es
        # fricción: el encoder dejó de contar o el lazo de la placa se colgó. Se corta antes y con
        # un motivo claro, en vez de esperar al corte por error de 45°.
        if th == getattr(self, "_th_prev", None) and abs(self.theta_cmd - th) > math.radians(5.0):
            if self._t_igual_desde is None:
                self._t_igual_desde = t
            elif t - self._t_igual_desde > 0.5:
                return (f"lectura del encoder congelada en {math.degrees(th):+.3f}° durante más de 0,5 s con "
                        f"{math.degrees(self.theta_cmd - th):+.1f}° de error (¿encoder o placa colgada?)")
        else:
            self._t_igual_desde = None
        self._th_prev = th
        # Error de seguimiento. Cambio del 2026-10-08 (c02, salida de Stanley): con la consigna a
        # ~510 °/s el volante iba a 730 °/s (vel_limit) hacia ella y acumuló 118° de atraso; eso es
        # saturación de tasa, no una falla, y el corte anterior (45° durante 0,5 s) lo detenía.
        # Ahora el error grande solo corta si el volante está quieto (|ω| < 90 °/s: trabado) o se
        # aleja de la consigna (desbocado). Si va hacia ella, se tolera hasta 4·corte_error (180°).
        err = self.theta_cmd - th
        quieto = abs(w) < math.radians(90.0)
        alejandose = (w * err) < 0 and not quieto
        if abs(err) > 4.0 * self.corte_error:
            return f"θ_cmd − θ_meas = {math.degrees(err):+.1f}° supera {math.degrees(4.0 * self.corte_error):.0f}°"
        if abs(err) > self.corte_error and (quieto or alejandose):
            if self.t_error_desde is None:
                self.t_error_desde = t
            elif t - self.t_error_desde > self.corte_error_s:
                return (f"θ_cmd − θ_meas = {math.degrees(err):+.1f}° durante más de {self.corte_error_s} s "
                        f"con ω = {math.degrees(w):+.0f} °/s (volante {'trabado' if quieto else 'desbocado'})")
        else:
            self.t_error_desde = None
        return None

    def _cortar(self, motivo):
        if self.corte is None:
            self.corte = motivo
        try:
            self.pl.center()   # motor a IDLE
        except Exception:
            pass
        if self.ffb is not None:
            try:
                self.ffb.fijar_nivel(0.0)
            except Exception:
                pass

    def _bucle_espejo(self):
        t_salud = time.perf_counter()
        t_elec = t_salud - self.electrico_cada_s
        while self._vivo:
            t = time.perf_counter()
            try:
                with self.lock:
                    if self.corte is None:
                        if self.carga is not None:
                            self.tau = self.carga.paso(t, self.estado_ac())
                            if self.ffb is not None:
                                self.tau = self.ffb.fijar_torque(self.tau)   # el que de verdad se pidió
                        if self.capa is not None:
                            self.theta_cmd = self.capa.paso(t, self.theta_pedido)
                        if self.interpolada and self._t_k is not None:
                            dt = t - self._t_k
                            dtk = min(dt, 0.1)          # no extrapola más de dos ciclos
                            th_r = self._theta_k + self._v_k * dtk
                            self.theta_cmd = max(-self.limite, min(self.limite, th_r))
                            v_ff = self._v_k if (dt < 0.08 and abs(th_r) < self.limite) else 0.0
                            self.pl.set_position(self.theta_cmd, self._tau_serie(), vel_rad_s=v_ff)
                        elif self.capa is None and (self.carga is None or self.ffb is not None):
                            self.pl.hold()
                        else:
                            self.pl.set_position(self.theta_cmd, self._tau_serie())
                    th, w, _, _ = self._leer_y_reflejar()
                    self.espejo.append((t, math.degrees(self.theta_cmd), math.degrees(th), math.degrees(w),
                                        math.degrees(self.theta_pedido), self._t_leido, self._t_vjoy, self.tau))
                    if self.corte is None:
                        motivo = self._revisar(t)
                        if motivo is None and t - t_salud > self.salud_cada_s:
                            t_salud = t
                            est, err = self.pl.salud()
                            if est != 8 or err != 0:
                                motivo = f"placa en estado {est}, axis0.error = {err}"
                            self._temp = self._leer_temperatura()
                        if motivo:
                            self._cortar(motivo)
                    if t - t_elec >= self.electrico_cada_s:
                        t_elec = t
                        self.electrico.append((t, self._leer_opcional(PROP_IQ),
                                               self._leer_opcional(PROP_VBUS), self._temp))
            except Exception as e:  # serial caído, etc.
                self._cortar(f"error en el hilo espejo: {e!r}")
                return
            espera = self.periodo - (time.perf_counter() - t)
            if espera > 0:
                time.sleep(espera)

    # --- interfaz de TrialSteer ---
    def enviar(self, delta_rad, u_long):
        if self.corte is not None:
            raise CorteHIL(self.corte)
        if not self._hilo.is_alive():
            raise CorteHIL("el hilo espejo se detuvo")
        t = time.perf_counter()
        theta = self.k * float(delta_rad)
        theta_banco = max(-self.limite, min(self.limite, theta))   # tope del banco
        if theta_banco != theta:
            self.n_limitados += 1
        if self.zona_muerta_volante > 0.0 and self.hil_activo:
            if abs(theta_banco - self._theta_filtrado) > self.zona_muerta_volante:
                self._theta_filtrado = theta_banco - math.copysign(self.zona_muerta_volante, theta_banco - self._theta_filtrado)
            theta_banco = self._theta_filtrado
        if u_long >= 0:
            gas, freno = int(u_long * AXIS_MAX), 0
        else:
            gas, freno = 0, int(-u_long * AXIS_MAX)
        if not self.hil_activo:
            est = self.estado_ac()
            cerca = abs(self.theta_meas - max(-self.limite, min(self.limite, theta))) < math.radians(5.0)
            if est is not None and est[4] >= self.arranque and cerca:   # sin salto al pasar a HIL
                if self._t_sobre is None:
                    self._t_sobre = t
                elif t - self._t_sobre >= self.arranque_s:
                    self.hil_activo = True
                    self.ciclo_inicio_hil = len(self.ciclos)
                    print(f"  HIL activo desde el ciclo {self.ciclo_inicio_hil} ({est[4]:.0f} km/h)")
            else:
                self._t_sobre = None
        if not self.hil_activo:
            # Arranque en SIL: el volante va hacia el comando a tasa limitada.
            dt = 0.05 if self._t_prev_envio is None else max(0.0, t - self._t_prev_envio)
            paso = self.tasa_arranque * dt
            theta_banco = self.theta_pedido + max(-paso, min(paso, theta_banco - self.theta_pedido))
        self._t_prev_envio = t
        with self.lock:
            self._theta_sin_tope = theta
            self.theta_pedido = theta_banco
            self.theta_cmd = theta_banco if self.capa is None else self.capa.paso(t, theta_banco)
            if self.interpolada:
                if self._t_k is not None and t > self._t_k:
                    vmax = self.corte_omega / 1.5          # = vel_limit
                    self._v_k = max(-vmax, min(vmax, (theta_banco - self._theta_k) / (t - self._t_k)))
                self._theta_k, self._t_k = theta_banco, t
                v_ff = self._v_k if abs(self.theta_cmd) < self.limite else 0.0
                self.pl.set_position(self.theta_cmd, self._tau_serie(), vel_rad_s=v_ff)
            else:
                self.pl.set_position(self.theta_cmd, self._tau_serie())
            th, w, norm, sat_eje = self._leer_y_reflejar()
            self.j.data.wAxisY = gas
            self.j.data.wAxisZRot = freno
            self.j.update()
        self.ciclos.append((len(self.ciclos), t, float(delta_rad), math.degrees(self.theta_cmd),
                            math.degrees(th), math.degrees(w), norm, math.degrees(theta), int(self.hil_activo)))
        return gas / float(AXIS_MAX), freno / float(AXIS_MAX), norm, sat_eje

    def _tau_serie(self):
        """Torque por el comando de posición: solo sin ffb (vía que Odrive-Wheel no respeta)."""
        return self.tau if (self.carga is not None and self.ffb is None) else None

    def centrar(self):
        """Lo llama TrialSteer al terminar: para el hilo, suelta el motor y centra vJoy."""
        self._vivo = False
        if self._hilo.is_alive() and threading.current_thread() is not self._hilo:
            self._hilo.join(timeout=1.0)
        if self.ffb is not None:
            try:
                self.ffb.cerrar()
            except Exception:
                pass
        try:
            self.pl.close()   # IDLE y restaura la configuración de la placa
        except Exception:
            pass
        self.j.data.wAxisX = 16384
        self.j.data.wAxisY = 0
        self.j.data.wAxisZRot = 0
        self.j.update()

    # --- registro ---
    COLUMNAS_CICLOS = ["ciclo", "t_perf_s", "delta_aplicado_rad", "theta_cmd_deg", "theta_meas_deg",  # + hil_activo al final
                       "omega_dps", "steer_norm_vjoy", "theta_pedido_deg", "hil_activo"]
    COLUMNAS_ESPEJO = ["t_perf_s", "theta_cmd_deg", "theta_meas_deg", "omega_dps", "theta_pedido_deg",
                       "t_leido_s", "t_vjoy_s", "tau_carga_nm"]
    COLUMNAS_ELECTRICO = ["t_perf_s", "iq_a", "vbus_v", "temp_fet_c"]
