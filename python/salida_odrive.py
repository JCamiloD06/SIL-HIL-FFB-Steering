# -*- coding: utf-8 -*-
"""
salida_odrive.py
Low-level motor driver and 200 Hz continuous position ramp profile.

Author: Juan Camilo Diaz-Lopez
Department of Mechanical and Mechatronics Engineering
Universidad Nacional de Colombia, Sede La Paz
Research Paper: Physical Steering Dynamics and Force-Feedback Loading
                in Autonomous Path Tracking
"""

# salida_odrive.py
#
# Misma interfaz que plataforma.actuador.SalidaVJoy: enviar(delta_rad, u_long)
# devuelve (gas, freno, norm, sat_eje) y centrar() suelta todo. Los
# controladores de TrialSteer no se tocan y no saben que hay un volante.
#
# Cadena HIL-U:
#   δ de rueda (rad) → θ_cmd = δ·k (volante) → placa Odrive-Wheel (lazo de posición a 8 kHz)
#   → encoder θ_meas → vJoy X = θ_meas/(rango/2) → Assetto Corsa
# con k = (rango/2) / rueda_rad_eje_completo. Con rango 360° y 0,403 rad, k ≈ 7,8;
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
# capa, la consigna pedida θ_pedido = k·δ pasa por la capa y la placa recibe θ_cmd = capa(θ_pedido),
# evaluada en cada ciclo de TrialSteer y en cada paso del hilo espejo (~5 ms). Sin capa,
# θ_cmd = θ_pedido, como antes.
#
# Registro eléctrico (plan Q1, apéndice A; 2026-10-08): el hilo espejo lee Iq y Vbus cada
# electrico_cada_s (0,1 s) y la temperatura del FET junto con la salud (0,5 s). Son lecturas
# opcionales: si una propiedad no existe en el firmware, se registra NaN y se deja de pedir
# tras 3 fallos seguidos, sin cortar la corrida. La versión del firmware va en config.
#
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
        return self.tau if (self.carga is not None and self.ffb is None) else None

    def centrar(self):
