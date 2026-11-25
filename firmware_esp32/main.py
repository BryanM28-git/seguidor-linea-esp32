# main.py — Robot seguidor de línea UMNG (ESP32 30 pines, MicroPython >= 1.21)
# Modos: MANUAL (app Streamlit: botones/voz, o control ESP32 por ESP-NOW) y AUTO (sigue línea,
# detecta colores, esquiva obstáculos). Telemetría JSON en http://<ip>/estado
import json
import math
import time
import gc

try:
    import asyncio
except ImportError:
    import uasyncio as asyncio

from machine import Pin, I2C, ADC
import config as C
from motores import Motores
from regleta import RegletaRC
from color import TCS3200
from brujula import Brujula, dif_angulo
from ultrasonido import HCSR05
from odometria import Odometria
from machine import PWM

ticks_ms = time.ticks_ms
ticks_diff = time.ticks_diff


def _limitar(v, lim):
    return lim if v > lim else (-lim if v < -lim else v)


class Robot:
    def __init__(self):
        self.mot = Motores(C.PIN_IN1, C.PIN_IN2, C.PIN_IN3, C.PIN_IN4, C.PWM_FREQ,
                           C.PWM_MIN, C.INVERTIR_IZQ, C.INVERTIR_DER, C.AJUSTE_RECTO)
        self.mot.parar()
        pines = tuple(reversed(C.PIN_QRE)) if getattr(C, "INVERTIR_REGLETA", False) else C.PIN_QRE
        self.reg = RegletaRC(pines, C.RC_TIMEOUT_US, C.UMBRAL_LINEA, C.LINEA_NEGRA)
        self.tcs = TCS3200(C.PIN_S2, C.PIN_S3, C.PIN_TCS_OUT)
        # ultrasonido opcional: con PIN_TRIG = None no hay detección de obstáculos
        self.hc = HCSR05(C.PIN_TRIG, C.PIN_ECHO, C.DIST_MAX) if C.PIN_TRIG is not None else None
        self.error_brujula = ""
        try:
            i2c = I2C(0, sda=Pin(C.PIN_SDA), scl=Pin(C.PIN_SCL), freq=400000)
            self.bru = Brujula(i2c, C.DECLINACION, C.BRUJULA_INVERTIR)
        except Exception as e:                       # el robot sigue funcionando sin brújula
            self.bru = None
            self.error_brujula = str(e)
        self.servo = None
        self.servo_ang = 0.0
        if C.PIN_SERVO is not None:
            self.servo = PWM(Pin(C.PIN_SERVO), freq=50, duty_u16=self._duty_servo(0))
        self._rumbo_prev = None
        self.tasa_giro = 0.0                         # grados/s (de la brújula)
        self._cmd_f = [0.0, 0.0]                     # comando de motores filtrado (memoria de giro)
        self.odo = Odometria(C.VEL_MM_S, C.VEL_BASE, C.PWM_MIN, C.ZONA_MUERTA, C.TROCHA_MM, C.TAU_MOTOR)
        self.anticipo = C.TAU_MOTOR                  # s; se ajusta solo en cal_giro
        self.adc_bat = None
        if C.PIN_BAT is not None:
            try:
                self.adc_bat = ADC(Pin(C.PIN_BAT))
                self.adc_bat.atten(ADC.ATTN_11DB)
            except Exception:
                self.adc_bat = None

        # estado general
        self.modo = "MANUAL"
        self.estado = "QUIETO"
        self.t_estado = 0
        self.calibrando = ""
        self.ip = ""
        self.vel_base = C.VEL_BASE
        self.vel_manual = C.VEL_MANUAL
        self.kp, self.kd, self.ki = C.KP, C.KD, C.KI
        self.lado = C.EVITAR_LADO
        # sensores
        self.rumbo = 0.0
        self.rumbo_deseado = 0.0
        self.dist = None
        self.color = "sin calibrar"
        self._color_cand = ""
        self._color_n = 0
        self.bateria = 0.0
        # control
        self.e_prev = 0.0
        self.i_acum = 0.0
        self.t_prev = 0
        self.ultimo_error = 0.0
        self.rumbo_linea = 0.0
        self.rumbo_ref = 0.0
        self.giro_ang = 0.0
        self.t_giro = 0
        self.dir_manual = 1
        self.t_fin = 0
        self.joy = (0.0, 0.0)
        self.t_joy = 0
        self.pausa_hasta = 0
        # registro
        self.obstaculos = 0
        self.historial_colores = []
        self.eventos = []
        self.hz = 0
        self._ciclos = 0
        self.cargar_calibracion()

    # ================================================================ utilidades
    def evento(self, txt):
        self.eventos.append([ticks_ms() // 100 / 10, txt])
        if len(self.eventos) > 25:
            self.eventos.pop(0)

    def cambiar(self, estado, now=None):
        self.estado = estado
        self.t_estado = ticks_ms() if now is None else now

    def guardar_calibracion(self):
        d = {"linea": self.reg.a_dict(), "color": self.tcs.ref,
             "brujula": self.bru.a_dict() if self.bru else None, "giro": self.odo.a_dict(),
             "anticipo": self.anticipo}
        try:
            with open(C.ARCHIVO_CALIB, "w") as f:
                json.dump(d, f)
        except OSError as e:
            self.evento("no se pudo guardar calibración: %s" % e)

    def cargar_calibracion(self):
        try:
            with open(C.ARCHIVO_CALIB) as f:
                d = json.load(f)
        except (OSError, ValueError):
            return
        self.reg.de_dict(d.get("linea"))
        self.tcs.ref = d.get("color") or {}
        if self.bru:
            self.bru.de_dict(d.get("brujula"))
        self.odo.de_dict(d.get("giro"))
        self.anticipo = d.get("anticipo", self.anticipo)

    # ================================================================ comandos
    def procesar(self, cmd):
        """Comandos de texto (HTTP, voz o ESP-NOW). Devuelve un dict de respuesta."""
        cmd = (cmd or "").strip().lower()
        nombre, _, arg = cmd.partition(":")
        now = ticks_ms()
        if self.calibrando and nombre not in ("parar", "manual"):
            return {"ok": False, "msg": "calibrando " + self.calibrando}
        if nombre in ("parar", "manual", "alto"):
            self.modo = "MANUAL"
            self.cambiar("QUIETO", now)
            self.mot.parar()
            self.calibrando = ""
        elif nombre == "auto":
            if not self.reg.calibrada:
                self.evento("aviso: regleta sin calibrar")
            self.modo = "AUTO"
            self.e_prev = self.reg.error()
            self.i_acum = 0.0
            self.t_prev = now
            self.pausa_hasta = 0
            self.cambiar("SEGUIR", now)
            self.evento("modo autónomo")
        elif nombre in ("adelante", "atras"):
            self.modo = "MANUAL"
            self.dir_manual = 1 if nombre == "adelante" else -1
            self.rumbo_deseado = self.rumbo
            self.t_fin = now + C.T_MANUAL_MS
            self.cambiar("MAN_AVANZA", now)
        elif nombre in ("izquierda", "derecha"):
            self.modo = "MANUAL"
            self._iniciar_giro(self.rumbo + (90 if nombre == "derecha" else -90), now)
            self.cambiar("MAN_GIRO", now)
        elif nombre in ("rumbo", "girar"):
            try:
                a = float(arg)
            except ValueError:
                return {"ok": False, "msg": "ángulo inválido"}
            self.modo = "MANUAL"
            self._iniciar_giro(a if nombre == "rumbo" else self.rumbo + a, now)
            self.cambiar("MAN_GIRO", now)
        elif nombre == "joy":
            try:
                x, y = [float(v) for v in arg.split(",")]
            except ValueError:
                return {"ok": False, "msg": "joy:x,y con valores -1..1"}
            self.modo = "MANUAL"
            self.joy = (_limitar(x, 1.0), _limitar(y, 1.0))
            self.t_joy = now
            if self.estado != "MAN_JOY":
                self.cambiar("MAN_JOY", now)
        elif nombre == "vel":
            try:
                v = _limitar(float(arg), 100)
            except ValueError:
                return {"ok": False, "msg": "vel:0..100"}
            self.vel_base = v
            self.vel_manual = v
        elif nombre == "pid":
            try:
                p = [float(v) for v in arg.split(",")]
                self.kp, self.kd = p[0], p[1]
                if len(p) > 2:
                    self.ki = p[2]
            except (ValueError, IndexError):
                return {"ok": False, "msg": "pid:kp,kd[,ki]"}
        elif nombre == "ajuste":                     # ajuste:-4 -> frena la rueda izquierda 4 %
            try:
                self.mot.ajuste = _limitar(float(arg), 20)
            except ValueError:
                return {"ok": False, "msg": "ajuste:-20..20"}
        elif nombre == "lado":
            self.lado = -1 if arg.strip() in ("-1", "izquierda") else 1
        elif nombre == "cal_linea":
            asyncio.create_task(self.calibrar_linea())
        elif nombre == "cal_giro":
            asyncio.create_task(self.calibrar_giro())
        elif nombre == "cero":
            self.odo.cero()
            if not self.bru:
                self.rumbo = 0.0
            self.rumbo_deseado = self.rumbo
        elif nombre == "cal_brujula":
            if not self.bru:
                return {"ok": False, "msg": "sin brújula: " + self.error_brujula}
            asyncio.create_task(self.calibrar_brujula())
        elif nombre == "cal_color":
            if arg not in C.COLORES:
                return {"ok": False, "msg": "colores válidos: " + ", ".join(C.COLORES)}
            ref = self.tcs.calibrar(arg)
            self.guardar_calibracion()
            self.evento("color %s calibrado %s" % (arg, ref))
        elif nombre == "borrar_color":
            self.tcs.ref.pop(arg, None)
            self.guardar_calibracion()
        elif nombre == "reiniciar_contadores":
            self.obstaculos = 0
            self.historial_colores = []
        elif nombre == "prueba":
            asyncio.create_task(self.prueba_motores())
        else:
            return {"ok": False, "msg": "comando desconocido: " + cmd}
        return {"ok": True, "cmd": cmd, "modo": self.modo, "estado": self.estado}

    # ================================================================ movimientos básicos
    def _iniciar_giro(self, objetivo, now):
        self.rumbo_deseado = objetivo % 360.0
        self.giro_ang = dif_angulo(self.rumbo_deseado, self.rumbo)
        self.t_giro = now

    def _girar(self, now):
        """Gira en el sitio hacia rumbo_deseado. Devuelve True al llegar."""
        if self.bru:
            err = dif_angulo(self.rumbo_deseado, self.rumbo)
            if abs(err) < C.TOL_GIRO or ticks_diff(now, self.t_giro) > 5000:
                self.mot.parar()
                return True
            v = _limitar(C.KP_GIRO * err, C.VEL_GIRO_MAX)
            self.mot.mover(v, -v)
            return False
        # sin brújula: lazo cerrado sobre el rumbo estimado, siempre a la velocidad calibrada
        err = dif_angulo(self.rumbo_deseado, self.rumbo)
        # corta antes: lo que el robot sigue girando por inercia (anticipo calibrado sobre la línea)
        margen = max(1.5, abs(self.odo.w_dps) * 0.012) + abs(self.odo.w_dps) * self.anticipo
        if abs(err) < margen or ticks_diff(now, self.t_giro) > 5000:
            self.mot.parar()
            return True
        v = C.VEL_GIRO_CAL if err > 0 else -C.VEL_GIRO_CAL
        self.mot.mover(v, -v)
        return False

    def _avanzar_con_rumbo(self, objetivo, vel):
        self.rumbo_deseado = objetivo % 360.0
        # con brújula usa el rumbo medido; sin ella, el estimado (calibrado con cal_giro)
        corr = _limitar(C.KP_RUMBO * dif_angulo(self.rumbo_deseado, self.rumbo), 40)
        self.mot.mover(vel + corr, vel - corr)

    def _pid(self, now):
        dt = ticks_diff(now, self.t_prev) / 1000.0
        self.t_prev = now
        e = self.reg.error()
        d = (e - self.e_prev) / dt if dt > 0 else 0.0
        self.i_acum = _limitar(self.i_acum + e * dt, 0.5)
        self.e_prev = e
        corr = self.kp * e + self.kd * d + self.ki * self.i_acum
        base = self.vel_base * (1.0 - 0.35 * abs(e))     # frena un poco en curvas
        self.mot.mover(base + corr, base - corr)
        for i, v in enumerate((self.mot.izq, self.mot.der)):
            self._cmd_f[i] += 0.08 * (v - self._cmd_f[i])
        self.ultimo_error = e
        # dirección deseada = rumbo actual + ángulo hacia el punto de la línea bajo la regleta
        ang = math.degrees(math.atan2(e * C.SEMIANCHO_REGLETA_MM, C.DIST_EJE_REGLETA_MM))
        self.rumbo_deseado = (self.rumbo + ang) % 360.0

    # ================================================================ ciclo de control
    def tick(self, now):
        self.reg.actualizar()
        if self.calibrando:
            return
        if self.modo == "AUTO":
            self._auto(now)
        else:
            self._manual(now)

    def _obstaculo(self, factor=1.0):
        return self.dist is not None and 0 < self.dist < C.DIST_OBSTACULO * factor

    def _auto(self, now):
        st = self.estado
        linea = self.reg.hay_linea
        dt = ticks_diff(now, self.t_estado)
        if st in ("SEGUIR", "PERDIDA", "BUSCAR") and self._obstaculo():
            self.obstaculos += 1
            self.evento("obstáculo %d a %.0f cm: esquivando" % (self.obstaculos, self.dist))
            base_ref = self.rumbo_linea if st != "SEGUIR" else self.rumbo
            self.rumbo_ref = (base_ref + self.servo_ang) % 360.0     # dirección hacia el obstáculo
            self._iniciar_giro(self.rumbo_ref + self.lado * 90, now)
            self.cambiar("EVITAR_GIRO", now)
            return
        if self.pausa_hasta and ticks_diff(self.pausa_hasta, now) > 0:
            self.mot.parar()
            return
        if st == "SEGUIR":
            if linea:
                self._pid(now)
                self.rumbo_linea = self.rumbo
            else:
                self.cambiar("PERDIDA", now)
        elif st == "PERDIDA":                        # p. ej. tramo de color que el IR no ve
            if linea:
                self._volver_a_seguir(now)
            elif dt > C.T_PERDIDA_MS:
                self.cambiar("BUSCAR", now)
            else:
                # memoria de giro: repite el comando promedio de antes (curva o recta)
                self.mot.mover(self._cmd_f[0], self._cmd_f[1])
        elif st == "BUSCAR":
            if linea:
                self._volver_a_seguir(now)
            elif dt > C.T_BUSCAR_MS:
                self.mot.parar()
                self.cambiar("DETENIDO", now)
                self.evento("línea perdida: detenido")
            else:
                s = 1 if self.ultimo_error >= 0 else -1
                self.mot.mover(s * 40, -s * 40)
        elif st == "EVITAR_GIRO":                    # 1) gira 90° hacia el lado de esquive
            if self._girar(now):
                self.cambiar("EVITAR_LADO", now)
        elif st == "EVITAR_LADO":                    # 2) se aparta de la línea
            if dt > self._ms(C.EVITAR_LATERAL_MM):
                self._iniciar_giro(self.rumbo_ref, now)
                self.cambiar("EVITAR_GIRO2", now)
            else:
                self._avanzar_con_rumbo(self.rumbo_deseado, self.vel_base)
        elif st == "EVITAR_GIRO2":                   # 3) vuelve al rumbo original
            if self._girar(now):
                self.cambiar("EVITAR_PASAR", now)
        elif st == "EVITAR_PASAR":                   # 4) avanza paralelo hasta rebasarlo
            if linea and dt > 200:                   # la pista curvó hacia el robot: la toma
                self.evento("línea recuperada tras obstáculo")
                self._volver_a_seguir(now)
            elif dt > self._ms(C.EVITAR_PASAR_MM):
                self._iniciar_giro(self.rumbo_ref - self.lado * 45, now)
                self.cambiar("EVITAR_GIRO3", now)
            else:
                self._avanzar_con_rumbo(self.rumbo_ref, self.vel_base)
        elif st == "EVITAR_GIRO3":                   # 5) apunta en diagonal hacia la línea
            if self._girar(now):
                self.cambiar("EVITAR_VOLVER", now)
        elif st == "EVITAR_VOLVER":                  # 6) avanza hasta cruzar la línea
            if linea:
                self.evento("línea recuperada tras obstáculo")
                self._volver_a_seguir(now)
            elif dt > C.EVITAR_TIMEOUT_MS:
                self.cambiar("BUSCAR", now)
            else:
                # si no cruza la línea donde se esperaba (p. ej. la pista curva), gira en espiral
                # hacia el lado de la línea hasta encontrarla
                esperado = self._ms(C.EVITAR_LATERAL_MM * 1.41 * 1.3)
                extra = 0 if dt < esperado else min(120, 60 * (dt - esperado) / 1000)
                self._avanzar_con_rumbo(self.rumbo_ref - self.lado * (45 + extra), self.vel_base * 0.8)
        else:                                        # DETENIDO
            self.mot.parar()

    def _ms(self, mm):
        """Tiempo para recorrer mm a la velocidad base actual."""
        v = C.VEL_MM_S * self.vel_base / C.VEL_BASE
        return int(1000 * mm / v) if v > 0 else 0

    def _volver_a_seguir(self, now):
        self.e_prev = self.reg.error()
        self.t_prev = now
        self.cambiar("SEGUIR", now)

    def _manual(self, now):
        st = self.estado
        if st == "MAN_AVANZA":
            if ticks_diff(now, self.t_fin) > 0:
                self.mot.parar()
                self.cambiar("QUIETO", now)
            elif self.dir_manual > 0 and self._obstaculo(0.8):
                self.mot.parar()
                self.cambiar("QUIETO", now)
                self.evento("parada por obstáculo a %.0f cm" % self.dist)
            else:
                self._avanzar_con_rumbo(self.rumbo_deseado, self.dir_manual * self.vel_manual)
        elif st == "MAN_GIRO":
            if self._girar(now):
                self.cambiar("QUIETO", now)
        elif st == "MAN_JOY":
            x, y = self.joy
            if ticks_diff(now, self.t_joy) > C.JOY_TIMEOUT_MS or (y > 0 and self._obstaculo(0.8)):
                self.mot.parar()
                self.cambiar("QUIETO", now)
            else:
                self.mot.mover(100 * y + 60 * x, 100 * y - 60 * x)
                self.rumbo_deseado = self.rumbo
        else:
            self.mot.parar()

    # ================================================================ sensores lentos
    def actualizar_tasa(self, now):
        if self._rumbo_prev is not None:
            dt = ticks_diff(now, self._rumbo_prev[1]) / 1000.0
            if dt > 0:
                t = dif_angulo(self.rumbo, self._rumbo_prev[0]) / dt
                self.tasa_giro += 0.25 * (t - self.tasa_giro)
        self._rumbo_prev = (self.rumbo, now)

    @staticmethod
    def _duty_servo(ang):
        us = 1500 + ang * 1000 / 90.0              # SG90: 500..2500 us
        return int(us * 65535 / 20000)

    def apuntar_servo(self):
        """Apunta el HC hacia la pista que viene: ángulo = distancia * curvatura / 2."""
        if not self.servo:
            return
        ang = 0.0
        v = C.VEL_MM_S * (self.mot.izq + self.mot.der) / 2 / C.VEL_BASE
        if self.modo == "AUTO" and self.estado in ("SEGUIR", "PERDIDA") and v > 40:
            curv = math.radians(self.tasa_giro) / v                 # 1/mm
            ang = math.degrees(C.SERVO_LOOKAHEAD_MM * curv / 2)
            ang = _limitar(ang, C.SERVO_MAX_ANG)
        self.servo_ang += 0.4 * (ang - self.servo_ang)
        self.servo.duty_u16(self._duty_servo(-self.servo_ang if C.SERVO_INVERTIR else self.servo_ang))

    def leer_color(self, now, leer=True):
        if leer:
            self.tcs.leer()
        nombre, _ = self.tcs.clasificar(dist_max=C.COLOR_DIST_MAX)
        if nombre == self._color_cand:
            self._color_n += 1
        else:
            self._color_cand, self._color_n = nombre, 1
        if self._color_n == C.COLOR_LECTURAS_ESTABLES and nombre != self.color:
            self.color = nombre
            if nombre not in ("desconocido", "sin calibrar"):
                self.historial_colores.append([ticks_ms() // 100 / 10, nombre])
                if len(self.historial_colores) > 30:
                    self.historial_colores.pop(0)
                acc = C.ACCIONES_COLOR.get(nombre)
                if acc and self.modo == "AUTO":
                    if acc[0] == "pausa":
                        self.pausa_hasta = now + acc[1]
                    elif acc[0] == "vel":
                        self.vel_base = acc[1]
                    self.evento("color %s: %s %s" % (nombre, acc[0], acc[1]))

    def leer_bateria(self):
        if self.adc_bat:
            try:
                self.bateria = round(self.adc_bat.read_uv() / 1e6 * C.BAT_DIVISOR, 2)
            except AttributeError:
                self.bateria = round(self.adc_bat.read_u16() / 65535 * 3.3 * C.BAT_DIVISOR, 2)

    async def tarea_sensores(self):
        k = 0
        while True:
            now = ticks_ms()
            try:
                if self.bru and not self.calibrando == "brujula":
                    self.rumbo = self.bru.leer()
                    self.actualizar_tasa(now)
                self.apuntar_servo()
                if self.hc and k % 3 == 0:         # HC-SR05 cada 60 ms
                    self.dist = self.hc.medir()
                if self.tcs.leer_paso():           # un filtro por vuelta: color completo cada 80 ms
                    self.leer_color(now, leer=False)
                if k % 50 == 0:
                    self.leer_bateria()
            except Exception as e:
                self.evento("error sensor: %s" % e)
            k += 1
            await asyncio.sleep_ms(20)

    async def tarea_control(self):
        t_hz = ticks_ms()
        t_ant = t_hz
        while True:
            t = ticks_ms()
            if not self.bru:                     # rumbo estimado con el comando que tuvo cada motor
                self.odo.actualizar(self.mot.izq, self.mot.der, ticks_diff(t, t_ant) / 1000.0)
                self.rumbo = self.odo.rumbo
                self.tasa_giro = self.odo.w_dps
            t_ant = t
            try:
                self.tick(t)
            except Exception as e:
                self.mot.parar()
                self.modo, self.estado = "MANUAL", "QUIETO"
                self.evento("error control: %s" % e)
            self._ciclos += 1
            if ticks_diff(t, t_hz) >= 1000:
                self.hz = self._ciclos
                self._ciclos = 0
                t_hz = t
            espera = 10 - ticks_diff(ticks_ms(), t)
            await asyncio.sleep_ms(espera if espera > 0 else 0)

    # ================================================================ calibraciones
    async def calibrar_linea(self):
        self.calibrando = "linea"
        self.evento("calibrando regleta: pon el robot sobre la línea")
        self.reg.reiniciar_calibracion()
        t0 = ticks_ms()
        while ticks_diff(ticks_ms(), t0) < 4250 and self.calibrando:
            # barrido simétrico: medio tramo a la derecha y luego tramos completos alternados,
            # así la línea pasa por debajo de los 8 sensores en ambos sentidos
            t = ticks_diff(ticks_ms(), t0)
            s = 1 if t < 250 else (-1 if ((t - 250) // 500) % 2 == 0 else 1)
            self.mot.mover(s * C.VEL_GIRO_CAL, -s * C.VEL_GIRO_CAL)
            self.reg.calibrar_muestra()
            await asyncio.sleep_ms(4)
        self.mot.parar()
        ok = self.reg.terminar_calibracion()
        self.guardar_calibracion()
        self.evento("regleta calibrada" if ok else "regleta: poco contraste, repite sobre la línea")
        self.calibrando = ""

    async def calibrar_giro(self):
        """Sin brújula: mide la velocidad real de giro. El robot debe estar centrado sobre un tramo
        RECTO de la línea, mirando a lo largo de ella. Al girar en el sitio, la regleta cruza la
        línea cada 180°; el tiempo entre cruces da los grados por segundo."""
        self.calibrando = "giro"
        self.evento("calibrando giro: robot centrado sobre una línea recta")
        cruces = []
        dentro = True                         # empieza sobre la línea: no cuenta el primer cruce
        t0 = ticks_ms()
        while ticks_diff(ticks_ms(), t0) < 9000 and self.calibrando and len(cruces) < 7:
            self.mot.mover(C.VEL_GIRO_CAL, -C.VEL_GIRO_CAL)
            self.reg.actualizar()
            if self.reg.hay_linea and abs(self.reg.error()) < 0.35 and not dentro:
                cruces.append(ticks_ms())
                dentro = True
            elif not self.reg.hay_linea:
                dentro = False
            await asyncio.sleep_ms(2)
        self.mot.parar()
        ok = False
        if len(cruces) >= 4:
            iv = [ticks_diff(cruces[i + 1], cruces[i]) for i in range(1, len(cruces) - 1)]
            media = sum(iv) / len(iv)
            ok = media > 0 and self.odo.calibrar_giro(C.VEL_GIRO_CAL, 180000.0 / media)
        if ok:
            self.evento("giro calibrado: %.0f °/s a %d %%" % (self.odo.grados_s, C.VEL_GIRO_CAL))
            # ajuste del anticipo: gira hasta centrarse en la línea, da media vuelta y mide cuánto se pasó
            for _ in range(3):
                if not await self._centrar_en_linea():
                    break
                await asyncio.sleep_ms(400)
                res = await self._giro_prueba(180)
                if res is None:
                    break
                w = max(abs(self.odo.grados_s), 1.0)
                self.anticipo = _limitar(self.anticipo + res / w, 0.4)
                self.evento("prueba 180°: se pasó %.1f° -> anticipo %.0f ms" % (res, self.anticipo * 1000))
            self.guardar_calibracion()
        else:
            self.evento("giro: no vio la línea suficientes veces (%d cruces)" % len(cruces))
        self.calibrando = ""

    async def _centrar_en_linea(self):
        """Gira despacio hasta que la regleta queda centrada sobre la línea."""
        t0 = ticks_ms()
        while ticks_diff(ticks_ms(), t0) < 6000 and self.calibrando:
            self.reg.actualizar()
            if self.reg.hay_linea and abs(self.reg.error()) < 0.12:
                self.mot.parar()
                return True
            self.mot.mover(C.VEL_GIRO_CAL * 0.8, -C.VEL_GIRO_CAL * 0.8)
            await asyncio.sleep_ms(3)
        self.mot.parar()
        return False

    async def _giro_prueba(self, ang):
        """Gira 'ang' grados con el control normal y devuelve cuánto se pasó (°, + = de más)."""
        self.reg.actualizar()
        e0 = self.reg.error()
        self._iniciar_giro(self.rumbo + ang, ticks_ms())
        while not self._girar(ticks_ms()):
            await asyncio.sleep_ms(5)
        await asyncio.sleep_ms(500)
        self.reg.actualizar()
        if not self.reg.hay_linea:
            return None
        # error de la regleta -> ángulo; al pasarse a la derecha, la línea queda a la izquierda (error < 0)
        a0 = math.degrees(math.atan2(e0 * C.SEMIANCHO_REGLETA_MM, C.DIST_EJE_REGLETA_MM))
        a1 = math.degrees(math.atan2(self.reg.error() * C.SEMIANCHO_REGLETA_MM, C.DIST_EJE_REGLETA_MM))
        sentido = 1 if self.giro_ang >= 0 else -1      # 180° es ambiguo: usa el sentido real del giro
        return (a0 - a1) * sentido

    async def calibrar_brujula(self):
        self.calibrando = "brujula"
        self.evento("calibrando brújula: el robot da 2 vueltas")
        self.bru.iniciar_calibracion()
        t0 = ticks_ms()
        while ticks_diff(ticks_ms(), t0) < 9000 and self.calibrando:
            self.mot.mover(42, -42)
            self.bru.muestra_calibracion()
            await asyncio.sleep_ms(20)
        self.mot.parar()
        ok = self.bru.terminar_calibracion()
        self.guardar_calibracion()
        self.evento("brújula calibrada" if ok else "brújula: no giró lo suficiente")
        self.calibrando = ""

    async def prueba_motores(self):
        self.calibrando = "prueba"
        for nombre, iz, de in (("izq adelante", 60, 0), ("izq atrás", -60, 0),
                               ("der adelante", 0, 60), ("der atrás", 0, -60)):
            self.evento("prueba: " + nombre)
            self.mot.mover(iz, de)
            await asyncio.sleep_ms(900)
            self.mot.parar()
            await asyncio.sleep_ms(300)
        self.calibrando = ""

    # ================================================================ telemetría
    def telemetria(self):
        return {
            "modo": self.modo, "estado": self.estado, "calibrando": self.calibrando,
            "rumbo": round(self.rumbo, 1), "rumbo_deseado": round(self.rumbo_deseado, 1),
            "brujula": self.bru.tipo if self.bru else "sin brújula",
            "rumbo_fuente": "brújula" if self.bru else "estimado (odometría)",
            "linea": self.reg.valores, "posicion": self.reg.posicion, "hay_linea": self.reg.hay_linea,
            "error_linea": round(self.ultimo_error, 3),
            "color": self.color, "rgb": self.tcs.rgb_aprox(), "frecuencias": self.tcs.ultimo,
            "distancia": self.dist, "obstaculos": self.obstaculos, "servo": round(self.servo_ang),
            "tasa_giro": round(self.tasa_giro, 1),
            "vel": [round(self.mot.izq), round(self.mot.der)],
            "bateria": self.bateria, "colores": self.historial_colores[-10:],
            "eventos": self.eventos[-8:], "hz": self.hz,
            "calibrado": {"linea": self.reg.calibrada, "brujula": bool(self.bru and self.bru.calibrada),
                          "giro": self.odo.calibrada,
                          "colores": sorted(self.tcs.ref.keys())},
            "pid": [self.kp, self.kd, self.ki], "vel_base": self.vel_base, "ajuste": self.mot.ajuste,
            "anticipo_ms": round(self.anticipo * 1000),
            "ip": self.ip, "t": ticks_ms(), "mem": gc.mem_free() if hasattr(gc, "mem_free") else 0,
        }

    # ================================================================ servidor HTTP
    async def manejar(self, r, w):
        try:
            primera = await r.readline()
            while True:
                h = await r.readline()
                if not h or h == b"\r\n":
                    break
            partes = primera.decode().split(" ")
            ruta = partes[1] if len(partes) > 1 else "/"
            path, _, qs = ruta.partition("?")
            q = parse_qs(qs)
            codigo = "200 OK"
            if path == "/estado":
                body = self.telemetria()
            elif path == "/cmd":
                body = self.procesar(q.get("c", ""))
            elif path == "/":
                body = {"robot": "seguidor UMNG", "rutas": ["/estado", "/cmd?c=..."]}
            else:
                codigo, body = "404 Not Found", {"ok": False}
            w.write(("HTTP/1.0 %s\r\nContent-Type: application/json\r\n"
                     "Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n" % codigo).encode())
            w.write(json.dumps(body).encode())
            await w.drain()
        except Exception as e:
            self.evento("error http: %s" % e)
        finally:
            try:
                w.close()
                await w.wait_closed()
            except Exception:
                pass

    async def tarea_espnow(self):
        """Control remoto: otro ESP32 envía los mismos comandos de texto (p. ej. b'joy:0.2,0.8')."""
        try:
            import aioespnow
        except ImportError:
            return
        try:
            e = aioespnow.AIOESPNow()
            e.active(True)
        except Exception as ex:
            self.evento("ESP-NOW no disponible: %s" % ex)
            return
        self.evento("ESP-NOW listo")
        async for _mac, msg in e:
            if msg:
                try:
                    self.procesar(msg.decode())
                except Exception:
                    pass


# ==================================================================== auxiliares
def _unquote(s):
    """Decodifica %XX y '+' de la URL (compatible con MicroPython)."""
    s = s.replace("+", " ")
    out = bytearray()
    i = 0
    while i < len(s):
        if s[i] == "%" and i + 3 <= len(s):
            try:
                out.append(int(s[i + 1:i + 3], 16))
                i += 3
                continue
            except ValueError:
                pass
        out.extend(s[i].encode())
        i += 1
    return out.decode()


def parse_qs(qs):
    d = {}
    for par in qs.split("&"):
        if "=" in par:
            k, v = par.split("=", 1)
            d[_unquote(k)] = _unquote(v)
    return d


def conectar_wifi():
    import network
    try:
        network.hostname(C.HOSTNAME)
    except Exception:
        pass
    sta = network.WLAN(network.STA_IF)
    sta.active(True)                     # también lo necesita ESP-NOW
    if C.WIFI_MODO == "STA":
        sta.connect(C.WIFI_SSID, C.WIFI_PASS)
        for _ in range(60):
            if sta.isconnected():
                ip = sta.ifconfig()[0]
                print("WiFi conectado. Abre http://%s  o  http://%s.local" % (ip, C.HOSTNAME))
                return ip
            time.sleep_ms(250)
        print("No se pudo conectar a", C.WIFI_SSID, "-> creando red propia")
        sta.disconnect()
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=C.AP_SSID, password=C.AP_PASS, authmode=3)
    ip = ap.ifconfig()[0]
    print("Red propia '%s' (clave %s). IP %s" % (C.AP_SSID, C.AP_PASS, ip))
    return ip


async def principal():
    robot = Robot()
    robot.ip = conectar_wifi()
    if robot.bru:
        robot.evento("brújula %s" % robot.bru.tipo)
    else:
        robot.evento("sin brújula: rumbo estimado por odometría")
    asyncio.create_task(robot.tarea_control())
    asyncio.create_task(robot.tarea_sensores())
    asyncio.create_task(robot.tarea_espnow())
    await asyncio.start_server(robot.manejar, "0.0.0.0", C.PUERTO)
    print("Servidor listo en el puerto", C.PUERTO)
    while True:
        await asyncio.sleep(2)
        gc.collect()


if __name__ == "__main__":
    try:
        asyncio.run(principal())
    finally:
        # deja los motores quietos si el programa se detiene (Ctrl+C en Thonny)
        for p in (C.PIN_IN1, C.PIN_IN2, C.PIN_IN3, C.PIN_IN4):
            Pin(p, Pin.OUT, value=0)
