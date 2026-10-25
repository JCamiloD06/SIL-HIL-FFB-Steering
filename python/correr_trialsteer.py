# correr_trialsteer.py
# Corre TrialSteer v1.0.1 (código congelado del paper 1) en SIL, HIL-U o HIL-L.
#
#   SIL   : TrialSteer tal cual. δ → vJoy (SalidaVJoy original).
#   HIL-U : δ → volante físico (Odrive-Wheel) → δ_meas → vJoy (SalidaODrive). Sin FFB.
#   HIL-L : HIL-U + torque de carga como feedforward en la misma placa (carga_hil.py, Q2-D-034):
#           --carga-fuente modelo|finalff --carga-k ... [--carga-signo-ff ±1] [--carga-tau-max 0.5]
#
# En las dos condiciones se registra además la señal de FFB de AC (finalFF, registro_ffb_ac.py)
# en ac_ffb.csv, y en HIL-U la corriente, el voltaje y la temperatura de la placa en
# odrive_electrico.csv (2026-10-08, plan Q1 apéndice A y Q2-D-034).
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
    """¿Está "ODrive-Wheel HID" entre los joysticks? (SDL2; si falla, se comprueba por PnP y por defecto False)."""
    try:
        import sdl2
        sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)
        try:
            return any(sdl2.SDL_JoystickNameForIndex(i) == b"ODrive-Wheel HID"
                       for i in range(sdl2.SDL_NumJoysticks()))
        finally:
            sdl2.SDL_QuitSubSystem(sdl2.SDL_INIT_JOYSTICK)
    except Exception:
        pass
    try:
        import subprocess
        out = subprocess.check_output(
            ["powershell", "-NoProfile", "-Command", "Get-PnpDevice -Class HIDClass -Status OK | Select-Object -ExpandProperty FriendlyName"],
            text=True, timeout=3
        )
        return "ODrive" in out
    except Exception:
        return False


def dispositivos_ac():
    """Orden de los joysticks de DirectInput en el último log de AC (el índice JOY de controls.ini)."""
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
    # Sin valor por omisión: el antiguo (360°, k ≈ 7,8) ya no es el aprobado (Q2-D-035: 900°,
    # k ≈ 19,5, = LOCK de AC) y el banco solo se ha probado hasta ±57°. En HIL-U es obligatorio.
    ap.add_argument("--rango-volante-deg", type=float, default=None,
                    help="ángulo de volante del eje de vJoy completo (HIL-U, obligatorio); 900 según Q2-D-035 (LOCK del volante vJoy en el controls.ini de AC)")
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
    try:
        import ejecutar_corrida as ec
        from plataforma import juego_ac
        from plataforma.registro import RegistroCorrida
        from registro_ffb_ac import RegistroFFB, COLUMNAS as COLUMNAS_FFB
    except ImportError:
        print("=" * 75)
        print("[NOTE] Hardware-in-the-Loop Orchestrator Environment Requirement")
        print("=" * 75)
        print("correr_trialsteer.py is the real-time execution orchestrator designed")
        print("to interface with Assetto Corsa (v1.16.4) and the TrialSteer telemetry")
        print("harness connected to the physical direct-drive ODrive steering test bench.")
        print("")
        print("For standalone computational simulation, model identification, and")
        print("controller evaluation without external hardware dependencies:")
        print("  1) Standalone Actuator-Aware MPC (Python QP optimization):")
        print("     python mpc_actuator_aware.py")
        print("  2) Standalone vehicle and steering dynamics simulations (MATLAB):")
        print("     matlab/simulacion_lazo_cerrado.m")
        print("     matlab/identificacion_actuador.m")
        print("     matlab/indice_compatibilidad_gamma.m")
        print("=" * 75)
        return
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
            sys.exit(f"{propios.condicion} necesita --rango-volante-deg (900 según Q2-D-035 (LOCK del volante vJoy en el controls.ini de AC)).")
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
        """Envoltorio que aplica una banda muerta continua al error lateral y de rumbo (Enfoque A)."""
        def __init__(self, base, zona_ey_m=0.0, zona_epsi_rad=0.0):
            self._base = base
            self.zona_ey_m = float(zona_ey_m)
            self.zona_epsi_rad = float(zona_epsi_rad)
            self.parametros = dict(getattr(base, "parametros", {}))
            self.parametros["zona_muerta_ey_m"] = self.zona_ey_m
            self.parametros["zona_muerta_epsi_deg"] = math.degrees(self.zona_epsi_rad)
            self.nombre = getattr(base, "nombre", "controlador")

        def __getattr__(self, item):
            return getattr(self._base, item)

        def reiniciar(self):
            if hasattr(self._base, "reiniciar"):
                self._base.reiniciar()

        @staticmethod
        def _banda(val, umbral):
            if umbral <= 0.0:
                return val
            if abs(val) <= umbral:
                return 0.0
            return val - math.copysign(umbral, val)

        def calcular(self, e):
            if (self.zona_ey_m <= 0.0 and self.zona_epsi_rad <= 0.0) or e.v_ms < 10.0:
                return self._base.calcular(e)
            from dataclasses import replace
            e_mod = replace(
                e,
                e_y_tras=self._banda(e.e_y_tras, self.zona_ey_m),
                e_psi_tras=self._banda(e.e_psi_tras, self.zona_epsi_rad),
                e_y_del=self._banda(e.e_y_del, self.zona_ey_m),
                e_psi_del=self._banda(e.e_psi_del, self.zona_epsi_rad),
            )
            return self._base.calcular(e_mod)

    from mpc_actuator_aware import MPCActuatorAware
    import controladores
    if MPCActuatorAware.nombre not in controladores.NOMBRES:
        controladores.NOMBRES = controladores.NOMBRES + (MPCActuatorAware.nombre,)
    if MPCActuatorAware.nombre not in ec.NOMBRES:
        ec.NOMBRES = ec.NOMBRES + (MPCActuatorAware.nombre,)

    crear_ctrl_orig = ec.crear_controlador

    def fabrica_controlador(nombre, cfg):
        if nombre == MPCActuatorAware.nombre:
            p_ctrl = dict(cfg["controladores"].get(MPCActuatorAware.nombre, cfg["controladores"]["mpc_cinematico"]))
            if propios.mpc_rddelta is not None:
                p_ctrl["Rddelta"] = float(propios.mpc_rddelta)
            ctrl = MPCActuatorAware(
                p_ctrl,
                ts_s=cfg["ts_s"],
                delta_max_rad=cfg["direccion"]["delta_max_rad"],
                tasa_max_rad_s=cfg["direccion"]["tasa_max_rad_s"],
            )
            print(f"  Controlador: Actuator-Aware MPC (Rddelta = {ctrl.Rdd:.2f}, G_a de 2.° orden + slew-rate 560°/s)")
        else:
            ctrl = crear_ctrl_orig(nombre, cfg)

        if propios.zona_muerta_carril_m > 0.0 or propios.zona_muerta_rumbo_deg > 0.0:
            ctrl = ControladorZonaMuerta(
                ctrl,
                zona_ey_m=propios.zona_muerta_carril_m,
                zona_epsi_rad=math.radians(propios.zona_muerta_rumbo_deg),
            )
        return ctrl

    ec.crear_controlador = fabrica_controlador

    if propios.zona_muerta_carril_m > 0.0 or propios.zona_muerta_rumbo_deg > 0.0:
        print(f"  Zona muerta en controlador (Enfoque A): carril = ±{propios.zona_muerta_carril_m * 100:.1f} cm, "
              f"rumbo = ±{propios.zona_muerta_rumbo_deg:.2f}°")

    # 3. Registro en la carpeta del paper, con la condición en el nombre y en el manifiesto.
    class RegistroPaper(RegistroCorrida):
        def __init__(self, directorio_base, id_corrida, config, ruta_config, meta):
            id_paper = f"{propios.condicion.replace('-', '')}_{id_corrida}"
            meta = dict(meta)
            meta["paper_q2"] = {
                "condicion": propios.condicion,
                "directorio_trialsteer_original": directorio_base,
                "argv_envoltorio": sys.argv,
                "sha256_envoltorio": sha256(os.path.abspath(__file__)),
                "sha256_salida_odrive": sha256(os.path.join(AQUI, "salida_odrive.py")),
                "sha256_capa_actuador": sha256(os.path.join(AQUI, "capa_actuador.py")),
                "sha256_odrive_plant": sha256(os.path.join(BANCO, "odrive_plant.py")),
                "sha256_registro_ffb_ac": sha256(os.path.join(AQUI, "registro_ffb_ac.py")),
                "sha256_mpc_actuator_aware": sha256(os.path.join(AQUI, "mpc_actuator_aware.py")),
                "version_trialsteer": os.path.basename(os.path.dirname(APP_TRIALSTEER)),
                "zona_muerta": {
                    "zona_muerta_carril_m": propios.zona_muerta_carril_m,
                    "zona_muerta_rumbo_deg": propios.zona_muerta_rumbo_deg,
                    "zona_muerta_volante_deg": propios.zona_muerta_volante_deg,
                },
            }
            super().__init__(DIR_CORRIDAS, id_paper, config, ruta_config, meta)

        def escribir(self, trazada, perfil, resumen, info_estatica):
            s = estado["salida"]
            if s is not None:
                self.meta["paper_q2"]["volante"] = s.config
                self.meta["paper_q2"]["corte_hil"] = s.corte
                self.meta["paper_q2"]["ciclos_en_tope_banco"] = s.n_limitados
                self.meta["paper_q2"]["ciclo_inicio_hil"] = s.ciclo_inicio_hil
                if s.corte:
                    self.nota(f"corte HIL, {s.corte}")
            rf = estado["ffb"]
            filas_ffb = list(rf.filas) if rf is not None else []
            # Si AC no corre, Windows crea la memoria vacía y se leen ceros: packetId no cambia.
            self.meta["paper_q2"]["ffb_ac"] = {"filas": len(filas_ffb),
                                               "paquetes_distintos": len({f[1] for f in filas_ffb}),
                                               "nota": rf.nota if rf else None}
            carpeta = super().escribir(trazada, perfil, resumen, info_estatica)
            if s is not None:
                escribir_csv(os.path.join(carpeta, "odrive_ciclos.csv"), s.COLUMNAS_CICLOS, s.ciclos)
                escribir_csv(os.path.join(carpeta, "odrive_espejo.csv"), s.COLUMNAS_ESPEJO, s.espejo)
                escribir_csv(os.path.join(carpeta, "odrive_electrico.csv"), s.COLUMNAS_ELECTRICO,
                             list(s.electrico))
            escribir_csv(os.path.join(carpeta, "ac_ffb.csv"), COLUMNAS_FFB, filas_ffb)
            return carpeta

    ec.RegistroCorrida = RegistroPaper

    # 4. TrialSteer.
    sys.argv = [os.path.join(APP_TRIALSTEER, "ejecutar_corrida.py")] + resto
    estado["ffb"] = RegistroFFB()
    try:
        ec.main()
    except Exception as e:
        print(f"\n!!! La corrida terminó por: {e}")
        print("    El registro se guardó si hubo al menos un ciclo.")
    finally:
        if estado["ffb"] is not None:
            estado["ffb"].detener()
            print(f"FFB de AC: {len(estado['ffb'].filas)} lecturas"
                  + (f" ({estado['ffb'].nota})" if estado["ffb"].nota else ""))
        if estado["ffb_hid"] is not None:
            try:
                estado["ffb_hid"].cerrar()
            except Exception:
                pass
        if estado["planta"] is not None:
            try:
                estado["planta"].close()
            except Exception:
                pass
        s = estado["salida"]
        if s is not None:
            print(f"Volante: {len(s.ciclos)} ciclos, {len(s.espejo)} lecturas del espejo, "
                  f"corte: {s.corte or 'ninguno'}")
        if propios.reiniciar_ac and not propios.no_preparar_ac:
            try:
                if juego_ac.juego_abierto():
                    juego_ac.cerrar_juego(cfg_juego, print)
            except Exception as e:
                print(f"Aviso al cerrar AC: {e}")


if __name__ == "__main__":
    main()
