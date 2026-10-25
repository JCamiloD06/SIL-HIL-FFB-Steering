# -*- coding: utf-8 -*-
"""
correr_trialsteer.py
Real-time multi-body simulation and control execution orchestrator.

Author: Juan Camilo Diaz-Lopez
Department of Mechanical and Mechatronics Engineering
Universidad Nacional de Colombia, Sede La Paz
Research Paper: Physical Steering Dynamics and Force-Feedback Loading
                in Autonomous Path Tracking
"""

# correr_trialsteer.py
# Corre TrialSteer v1.0.1 (código congelado del paper 1) en SIL, HIL-U o HIL-L.
#
#   SIL   : TrialSteer tal cual. δ → vJoy (SalidaVJoy original).
#   HIL-U : δ → volante físico (Odrive-Wheel) → δ_meas → vJoy (SalidaODrive). Sin FFB.
#           --carga-fuente modelo|finalff --carga-k ... [--carga-signo-ff ±1] [--carga-tau-max 0.5]
#
# En las dos condiciones se registra además la señal de FFB de AC (finalFF, registro_ffb_ac.py)
# en ac_ffb.csv, y en HIL-U la corriente, el voltaje y la temperatura de la placa en
#
# No se modifica ningún archivo de scripts/acq/TrialSteer_v1.0.1. Este script
# importa su ejecutar_corrida.py y, antes de llamar a main(), cambia solo dos
# nombres de ese módulo:
#   SalidaVJoy      → SalidaODrive (solo en HIL-U)
#   RegistroCorrida → RegistroPaper (en ambas condiciones): guarda en
#                     data/raw/trialsteer/ del paper, antepone la condición al
#                     nombre de la corrida y agrega al manifiesto la condición,
#                     la configuración del volante y las huellas de estos scripts.
# El manifiesto de TrialSteer guarda además la huella de cada .py de TrialSteer,
# que prueba que los controladores no cambiaron.
#
# Uso, desde la raíz del paper (Assetto Corsa lo abre y prepara TrialSteer):
#   py -3.14 scripts\acq\hil\correr_trialsteer.py --condicion SIL   --controlador stanley --perfil nominal --sesion 0
#   py -3.14 scripts\acq\hil\correr_trialsteer.py --condicion HIL-U --port COM6 --rango-volante-deg 900 --controlador stanley --perfil nominal --sesion 0
# Cualquier otro argumento se pasa a TrialSteer (--fase, --etiqueta, --tiempo-max-s, ...).

import argparse
import csv
import hashlib
import json
import math
import os
import sys
import tempfile
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ_PAPER = os.path.dirname(os.path.dirname(os.path.dirname(AQUI)))
APP_TRIALSTEER = os.path.join(RAIZ_PAPER, "scripts", "acq", "TrialSteer_v1.0.1", "TrialSteer")
BANCO = os.path.join(RAIZ_PAPER, "scripts", "acq", "banco")
DIR_CORRIDAS = os.path.join(RAIZ_PAPER, "data", "raw", "trialsteer")

sys.path.insert(0, AQUI)
from capa_actuador import agregar_argumentos, desde_argumentos  # noqa: E402
import carga_hil  # noqa: E402


def sha256(ruta):
    with open(ruta, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def odrive_hid_conectado():
    ruta = os.path.join(os.path.expanduser("~"), "Documents", "Assetto Corsa", "logs", "log.txt")
    try:
        with open(ruta, encoding="utf-8", errors="replace") as f:
            return [l.split(":", 1)[1].strip() for l in f if l.startswith("DIRECT INPUT DEVICE:")]
    except OSError:
        return []


def escribir_csv(ruta, columnas, filas):
    with open(ruta, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(columnas)
        w.writerows(filas)


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--condicion", required=True, choices=["SIL", "HIL-U", "HIL-L"])
    ap.add_argument("--port", help="puerto COM de la placa (HIL-U)")
    # k ≈ 19,5, = LOCK de AC) y el banco solo se ha probado hasta ±57°. En HIL-U es obligatorio.
    ap.add_argument("--rango-volante-deg", type=float, default=None,
                    help="ángulo de volante del eje de vJoy completo (HIL-U, obligatorio); 900 según")
    ap.add_argument("--vel-limit-tps", type=float, default=2.0, help="vel_limit de la placa (HIL-U); 2 vueltas/s = 720 °/s (decisión del investigador, 2026-10-07)")
    ap.add_argument("--no-preparar-ac", action="store_true",
                    help="no abrir ni preparar Assetto Corsa (debe estar en pista y con vJoy al mando)")
    ap.add_argument("--reiniciar-ac", action="store_true", help="cerrar y reabrir AC al preparar")
    ap.add_argument("--arranque-sil-kmh", type=float, default=40.0,
                    help="HIL: AC recibe el comando (como en SIL) hasta pasar esta velocidad 1 s; 0 = HIL desde el inicio")
    ap.add_argument("--ganancias", type=float, nargs=3, metavar=("POS", "VEL", "VEL_INT"),
                    help="ganancias del servo solo para esta corrida (sin guardar); por omisión las de la placa (S6)")
    ap.add_argument("--consigna-interpolada", action="store_true",
                    help="consigna del volante interpolada con vel_ff en vez de escalones de 20 Hz (s21)")
    ap.add_argument("--zona-muerta-carril-m", type=float, default=0.0,
                    help="tolerancia de error lateral e_y en metros (Enfoque A: p.ej. 0.05 = 5 cm)")
    ap.add_argument("--zona-muerta-rumbo-deg", type=float, default=0.0,
                    help="tolerancia de error de rumbo e_psi en grados (Enfoque A: p.ej. 0.5°)")
    ap.add_argument("--zona-muerta-volante-deg", type=float, default=0.0,
                    help="tolerancia en grados de volante físico (Enfoque B: p.ej. 3.0°)")
    ap.add_argument("--mpc-rddelta", type=float, default=None,
                    help="peso Rddelta de penalización de tasa en MPC (ej: 15.0 para eliminar microcorrecciones)")
    ap.add_argument("--pista", default="monza", choices=["monza", "magione", "laguna_seca", "vallelunga_club"],
                    help="pista de ensayo (monza o magione, por omision monza)")
    ap.add_argument("--plantilla-trialsteer", action="store_true",
                    help="usar la plantilla de AC de TrialSteer en vez de configs/sesion_ac_paper")
    ap.add_argument("--auto-cero", action="store_true",
                    help="auto-centrado suave desde referencia_encoder.json sin pedir confirmación manual (HIL-U y HIL-L)")
    agregar_argumentos(ap)   # --capa-*: dinámica agregada en el PC para el barrido (HIL-U y HIL-L)
    carga_hil.agregar_argumentos(ap)   # --carga-*: solo HIL-L
    propios, resto = ap.parse_known_args()
    fisico = propios.condicion in ("HIL-U", "HIL-L")
    if fisico and not propios.port:
        ap.error(f"{propios.condicion} necesita --port")
    if propios.condicion == "HIL-L" and propios.carga_fuente is None:
        ap.error("HIL-L necesita --carga-fuente y --carga-k")
    if propios.condicion != "HIL-L" and propios.carga_fuente is not None:
        ap.error("--carga-* solo se usa con --condicion HIL-L")

    sys.path.insert(0, APP_TRIALSTEER)
    sys.path.insert(0, BANCO)
    sys.path.insert(0, AQUI)
    import ejecutar_corrida as ec
    from plataforma import juego_ac
    from plataforma.registro import RegistroCorrida

    from registro_ffb_ac import RegistroFFB, COLUMNAS as COLUMNAS_FFB
    estado = {"salida": None, "planta": None, "ffb": None, "ffb_hid": None}

    # Si es HIL-L, abrimos FFBHid ANTES de lanzar Assetto Corsa y sin doble SDL_Init,
    # para que DirectInput/SDL2 adquiera el dispositivo HID sin contienda con acs.exe.
    carga = carga_hil.desde_argumentos(propios) if fisico else None
    if carga is not None:
        if juego_ac.juego_abierto():
            try:
                juego_ac.cerrar_juego(juego_ac.cargar_config(), print)
            except Exception:
                pass
        from ffb_hid import FFBHid   # torque por el FFB USB HID de Odrive-Wheel (opción A)
        print("Inicializando FFB USB HID (ODrive-Wheel HID)...")
        estado["ffb_hid"] = FFBHid()
        print(f"  FFB HID listo: {estado['ffb_hid'].cal['nm_volante_por_nivel']:+.3f} N·m de volante por nivel")

    # 1. Assetto Corsa: mismo procedimiento que el botón Iniciar corrida del lanzador.
    info_ac = None
    if not propios.no_preparar_ac and "--info-sesion-ac" not in resto:
        if propios.pista == "magione":
            cfg_juego = juego_ac.cargar_config(os.path.join(RAIZ_PAPER, "configs", "magione", "juego_ac_magione.json"))
            plantilla_paper = os.path.join(RAIZ_PAPER, "configs", "sesion_ac_magione")
        elif propios.pista == "laguna_seca":
            cfg_juego = juego_ac.cargar_config(os.path.join(RAIZ_PAPER, "configs", "laguna_seca", "juego_ac_laguna_seca.json"))
            plantilla_paper = os.path.join(RAIZ_PAPER, "configs", "sesion_ac_laguna_seca")
        elif propios.pista == "vallelunga_club":
            cfg_juego = juego_ac.cargar_config(os.path.join(RAIZ_PAPER, "configs", "vallelunga_club", "juego_ac_vallelunga_club.json"))
            plantilla_paper = os.path.join(RAIZ_PAPER, "configs", "sesion_ac_vallelunga_club")
        else:
            cfg_juego = juego_ac.cargar_config()
            plantilla_paper = os.path.join(RAIZ_PAPER, "configs", "sesion_ac_paper")
        # Plantilla de AC según los dispositivos conectados (2026-10-08). AC numera los joysticks por
        # el orden de DirectInput y, con Odrive-Wheel conectado, la placa es el 0 y vJoy el 1 (log de
        # AC). La plantilla de TrialSteer pide JOY=0 para dirección, acelerador y freno: AC leía la
        # placa, el lanzador no veía el freno de prueba y AC mandaba su FFB al volante físico.
        # configs/sesion_ac_paper es la de TrialSteer con JOY=1 en esas tres secciones. TrialSteer
        # no se toca; la huella de la plantilla usada queda en la preparación.
        orden_prev = dispositivos_ac()
        idx_vjoy_prev = orden_prev.index("vJoy Device") if "vJoy Device" in orden_prev else 0
        if idx_vjoy_prev == 1 and not propios.plantilla_trialsteer:
            from pathlib import Path
            cfg_juego["_dir_plantilla"] = Path(plantilla_paper)
            joy_esperado = 1
            print(f"vJoy en indice 1 de AC: plantilla con JOY=1 ({plantilla_paper})")
        else:
            joy_esperado = 0
            print(f"vJoy en indice 0 de AC: plantilla con JOY=0")
        try:
            info_ac = juego_ac.preparar(cfg_juego, reiniciar=propios.reiniciar_ac)
        finally:
            orden = dispositivos_ac()
            if orden:
                idx = orden.index("vJoy Device") if "vJoy Device" in orden else None
                print(f"Joysticks según el log de AC: {orden}; vJoy = {idx}, la plantilla usa {joy_esperado}"
                      + ("" if idx == joy_esperado else "  <-- NO COINCIDEN: AC no leerá vJoy"))
        ruta_info = os.path.join(tempfile.gettempdir(), f"trialsteer_paper_sesion_{os.getpid()}.json")
        with open(ruta_info, "w", encoding="utf-8") as f:
            json.dump(info_ac, f, ensure_ascii=False, default=str)
        resto += ["--info-sesion-ac", ruta_info]

    # 2. Volante físico (HIL-U): se centra a mano y se arma justo antes del bucle.
    if fisico:
        if propios.rango_volante_deg is None:
            sys.exit(f"{propios.condicion} necesita --rango-volante-deg (900 según).")
        from odrive_plant import ODriveSteeringPlant
        from salida_odrive import SalidaODrive
        capa = desde_argumentos(propios)
        if capa is not None:
            print(f"  Capa de dinámica en el PC: {capa.config}")
        if carga is None:
            print("HIL-U. Volante físico en el lazo, sin force feedback.")
        else:
            print(f"HIL-L. Volante físico en el lazo con carga: {carga.config}")
            print(f"  FFB HID: {estado['ffb_hid'].cal['nm_volante_por_nivel']:+.3f} N·m de volante por nivel")
        print("  - corte de 24 V a la mano; encoder calibrado en este encendido (etapa 3)")
        print("  - no toques el volante durante la vuelta")
        estado["planta"] = ODriveSteeringPlant(propios.port, vel_limit_tps=propios.vel_limit_tps, armar=False,
                                               ganancias=propios.ganancias, auto_cero=propios.auto_cero,
                                               pedir_centro=not propios.auto_cero)

        def fabrica_salida(rueda_rad_eje_completo, signo_vjoy=1, id_dispositivo=1):
            estado["planta"].armar()
            if carga is not None:   # el signo de la carga sigue al de vJoy de TrialSteer
                carga.signo_vjoy = int(signo_vjoy)
                carga.config["signo_vjoy"] = carga.signo_vjoy
            s = SalidaODrive(rueda_rad_eje_completo, signo_vjoy, estado["planta"],
                             rango_volante_deg=propios.rango_volante_deg, id_dispositivo=id_dispositivo,
                             # Corte por velocidad ligado a vel_limit (1,2 = tolerancia de sobrevelocidad
                             # por omisión de ODrive); antes era 400 °/s fijo con vel_limit de 1 vuelta/s.
                             corte_omega_dps=1.5 * 360.0 * propios.vel_limit_tps, capa=capa,   # = TOLERANCIA_VEL
                             carga=carga, estado_ac=lambda: estado["ffb"].ultimo(),
                             arranque_sil_kmh=propios.arranque_sil_kmh or None,
                             consigna_interpolada=propios.consigna_interpolada,
                             zona_muerta_volante_deg=propios.zona_muerta_volante_deg,
                             ffb=estado.get("ffb_hid"))
            estado["salida"] = s
            print(f"  Volante armado. k = {s.k:.2f}° de volante por grado de rueda."
                  + (f" Zona muerta volante: ±{propios.zona_muerta_volante_deg:.1f}°" if propios.zona_muerta_volante_deg > 0 else ""))
            return s

        ec.SalidaVJoy = fabrica_salida

    class ControladorZonaMuerta:
