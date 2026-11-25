"""
Simulación del robot seguidor con el firmware REAL (firmware_seguidor/*.py) corriendo en el PC.
Se reemplazan machine/time por versiones simuladas conectadas a un mundo 2D:
pista con tramos de color, obstáculos, cinemática diferencial, brújula con offset magnético,
regleta RC leída por registros GPIO, TCS3200 por time_pulse_us y HC-SR05.
"""
import math, os, random, sys, time as _time, types, json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "stubs"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "firmware_esp32"))
import numpy as np

# ------------------------------------------------------------------ reloj simulado
class Reloj:
    us = 0
R = Reloj()
def ticks_us():
    R.us += 6                      # cada lectura del reloj "cuesta" ~6 us (bucle MicroPython nativo)
    return R.us
_time.ticks_us = ticks_us
_time.ticks_ms = lambda: R.us // 1000
_time.ticks_diff = lambda a, b: a - b
_time.sleep_us = lambda n: setattr(R, "us", R.us + int(n))
_time.sleep_ms = lambda n: setattr(R, "us", R.us + int(n) * 1000)

import asyncio
class _Espera:
    def __init__(self, ms): self.ms = ms
    def __await__(self):
        yield self.ms
asyncio.sleep_ms = lambda ms: _Espera(ms)

import machine
import config as C

# ------------------------------------------------------------------ pista
def construir_pista(segs, x=-300.0, y=-250.0, th=0.0, paso=2.0):
    P = [(x, y, th)]
    for s in segs:
        if s[0] == "S":
            for _ in range(int(round(s[1] / paso))):
                x += paso * math.cos(th); y += paso * math.sin(th); P.append((x, y, th))
        else:
            ang, Rr = math.radians(s[1]), s[2]
            n = int(round(abs(ang) * Rr / paso)); dth = ang / n
            for _ in range(n):
                th += dth / 2; x += paso * math.cos(th); y += paso * math.sin(th); th += dth / 2
                P.append((x, y, th))
    return np.array(P)

OSCURIDAD_IR = {"negro": 1.0, "azul": 0.8, "verde": 0.75, "rojo": 0.12, "amarillo": 0.08, "blanco": 0.0}
REFLEJO = {"negro": (0.05, 0.05, 0.06), "blanco": (0.92, 0.9, 0.88), "rojo": (0.82, 0.16, 0.14),
           "amarillo": (0.86, 0.78, 0.14), "verde": (0.14, 0.58, 0.26), "azul": (0.13, 0.25, 0.72)}

class Mundo:
    def __init__(self, pista, tramos, obstaculos, ruido=True, offset_mag=(320, -180)):
        self.P = pista
        self.tramos = tramos             # [(s0, s1, color)]
        self.obst = obstaculos           # [(x, y, r)]
        self.x, self.y, self.th = pista[0][0] - 107.0, pista[0][1], pista[0][2]
        self.vl = self.vr = 0.0
        self.B = 2 * 64.3                # trocha (mm)
        self.ruido = ruido
        self.off = offset_mag
        self.t_carga = 0
        self.rc_t = [2500] * 8
        self.choques = 0
        self.robot = None
        self.hay_brujula = True
        self.traza = []
        self.qre_y = [33.3 - i * 9.525 for i in range(8)]   # sensor 1 a la izquierda (+y)

    # geometría
    def a_mundo(self, px, py):
        c, s = math.cos(self.th), math.sin(self.th)
        return self.x + px * c - py * s, self.y + px * s + py * c
    def cercano(self, x, y):
        d2 = (self.P[:, 0] - x) ** 2 + (self.P[:, 1] - y) ** 2
        i = int(np.argmin(d2)); return math.sqrt(d2[i]), i * 2.0
    def color_en(self, s):
        for s0, s1, col in self.tramos:
            if s0 <= s < s1: return col
        return "negro"
    def cobertura(self, x, y, medio=9.5, borde=3.0):
        d, s = self.cercano(x, y)
        return max(0.0, min(1.0, (medio + borde / 2 - d) / borde)), s

    # periféricos
    def pin_a_entrada(self, n):
        if n in C.PIN_QRE and n == C.PIN_QRE[-1]:
            # se acaba de soltar el último pin de la regleta: calcula los tiempos de descarga
            self.t_carga = R.us
            for i in range(8):
                x, y = self.a_mundo(C.DIST_EJE_REGLETA_MM, self.qre_y[i])
                cov, s = self.cobertura(x, y)
                osc = cov * OSCURIDAD_IR[self.color_en(s)]
                self.rc_t[i] = 180 + osc * 2100 + (random.gauss(0, 25) if self.ruido else 0)
    def registro_gpio(self, addr):
        dt = R.us - self.t_carga
        v0 = v1 = 0
        for i, p in enumerate(C.PIN_QRE):
            if dt < self.rc_t[i]:
                if p < 32: v0 |= 1 << p
                else: v1 |= 1 << (p - 32)
        return v0 if addr == 0x3FF4403C else v1
    def magnetometro(self):
        k = 1500
        n = (lambda: random.gauss(0, 12)) if self.ruido else (lambda: 0)
        return (int(k * math.sin(self.th) + self.off[0] + n()), int(0.85 * k * math.cos(self.th) + self.off[1] + n()), 400)
    def pulso(self, pin, nivel, timeout):
        if pin == C.PIN_ECHO:
            d = self.distancia_cm()
            if d is None: return -2
            t = int(d * 58)
            return t if t <= timeout else -2
        if pin == C.PIN_TCS_OUT:
            s2 = machine.Pin._todos[C.PIN_S2]._v; s3 = machine.Pin._todos[C.PIN_S3]._v
            f = {(0, 0): 0, (1, 1): 1, (0, 1): 2, (1, 0): 3}[(s2, s3)]
            hz = self.freq_tcs()[f]
            R.us += int(1e6 / hz)                       # el pulso consume tiempo real
            return int(5e5 / hz)
        return -1
    def freq_tcs(self):
        x, y = self.a_mundo(C.TCS_X if hasattr(C, "TCS_X") else 78.5, 0)
        cov, s = self.cobertura(x, y, borde=6.0)
        lin, fondo = REFLEJO[self.color_en(s)], REFLEJO["blanco"]
        rgb = [cov * a + (1 - cov) * b for a, b in zip(lin, fondo)]
        k = 16000 * (1 + (random.gauss(0, 0.02) if self.ruido else 0))
        r, g, b = [max(40.0, k * v) for v in rgb]
        return (r, g, b, (r + g + b) / 3 * 1.25)
    def distancia_cm(self):
        fx, fy = self.a_mundo(167, 0)
        rob = getattr(self, "robot", None)
        haz = self.th - math.radians(rob.servo_ang if (rob and rob.servo) else 0.0)   # servo + = derecha
        best = None
        for ox, oy, r in self.obst:
            dx, dy = ox - fx, oy - fy
            dist = math.hypot(dx, dy)
            ang = math.degrees(math.atan2(dy, dx)) - math.degrees(haz)
            ang = (ang + 180) % 360 - 180
            if abs(ang) < 15 and dist - r < C.DIST_MAX * 10:
                d = (dist - r) / 10
                best = d if best is None else min(best, d)
        return best

    # dinámica
    def vel_rueda(self, pct, duty):
        if duty == 0: return 0.0
        d = duty / 655.35
        return math.copysign(max(0.0, d - 22) / 78 * 520, pct)
    def paso(self, dt, mot):
        tl = self.vel_rueda(mot.izq * mot.inv[0], mot._duty(mot.izq)) * mot.inv[0]
        tr = self.vel_rueda(mot.der * mot.inv[1], mot._duty(mot.der)) * mot.inv[1]
        a = min(1.0, dt / 0.08)                      # retardo mecánico 80 ms
        self.vl += a * (tl - self.vl); self.vr += a * (tr - self.vr)
        v = (self.vl + self.vr) / 2; w = (self.vr - self.vl) / self.B
        self.x += v * math.cos(self.th) * dt; self.y += v * math.sin(self.th) * dt; self.th += w * dt
        for ox, oy, r in self.obst:                  # choque con el cuerpo (rectángulo)
            c, s = math.cos(-self.th), math.sin(-self.th)
            bx, by = (ox - self.x) * c - (oy - self.y) * s, (ox - self.x) * s + (oy - self.y) * c
            qx, qy = min(max(bx, -66), 175), min(max(by, -77), 77)
            if math.hypot(bx - qx, by - qy) < r: self.choques += 1

MUNDO = None
def nuevo_mundo(**k):
    global MUNDO
    MUNDO = Mundo(**k); machine.MUNDO = MUNDO
    return MUNDO

def correr_corrutina(coro, robot, mundo):
    """Ejecuta una corrutina del firmware (calibraciones) avanzando el mundo exactamente el
    tiempo simulado transcurrido (esperas + lo que tardan las lecturas)."""
    ult = R.us
    try:
        ms = coro.send(None)
        while True:
            R.us += ms * 1000
            while R.us - ult >= 2000:
                if not robot.bru:                  # tarea_control sigue integrando la odometría
                    robot.odo.actualizar(robot.mot.izq, robot.mot.der, 0.002)
                    robot.rumbo = robot.odo.rumbo
                    robot.tasa_giro = robot.odo.w_dps
                mundo.paso(0.002, robot.mot); ult += 2000
            ms = coro.send(None)
    except StopIteration:
        pass

def bucle(robot, mundo, segundos, registrar=None):
    """Bucle principal: control cada 10 ms y sensores cada 40 ms (como tarea_control/tarea_sensores)."""
    pasos = int(segundos * 100)
    t0 = R.us
    for k in range(pasos):
        R.us = t0 + k * 10000
        now = R.us // 1000
        if k % 2 == 0:                                  # igual que tarea_sensores (cada 20 ms)
            j = k // 2
            if robot.bru:
                robot.rumbo = robot.bru.leer()
                robot.actualizar_tasa(now)
            robot.apuntar_servo()
            if robot.hc and j % 3 == 0: robot.dist = robot.hc.medir()
            if robot.tcs.leer_paso(): robot.leer_color(now, leer=False)
            if j % 50 == 0: robot.leer_bateria()
            R.us = t0 + k * 10000
        if not robot.bru:                                # igual que tarea_control sin brújula
            robot.odo.actualizar(robot.mot.izq, robot.mot.der, 0.01)
            robot.rumbo = robot.odo.rumbo
            robot.tasa_giro = robot.odo.w_dps
        robot.tick(now)
        mundo.paso(0.01, robot.mot)
        d, s = mundo.cercano(*mundo.a_mundo(0, 0))
        mundo.traza.append((mundo.x, mundo.y, robot.estado, s))
        if registrar: registrar(k, robot, mundo)
