# odometria.py — Rumbo estimado SIN brújula, integrando el comando de los motores
# Modelo: velocidad de rueda proporcional al duty por encima de la zona muerta, con un
# retardo de primer orden (inercia). La "trocha efectiva" se calibra girando sobre la línea.
import math


class Odometria:
    def __init__(self, vel_mm_s, vel_base, pwm_min, zona_muerta, trocha_mm, tau=0.08):
        self.vel_mm_s = vel_mm_s          # velocidad medida a vel_base
        self.vel_base = vel_base
        self.pwm_min = pwm_min
        self.zm = zona_muerta
        self.b = trocha_mm                # trocha efectiva (mm), se ajusta con calibrar_giro()
        self.tau = tau
        self.vl = 0.0
        self.vr = 0.0
        self.rumbo = 0.0                  # grados, horario, 0 = dirección inicial
        self.w_dps = 0.0
        self.x = 0.0
        self.y = 0.0
        self.calibrada = False
        self.grados_s = 0.0

    def _duty(self, pct):
        a = min(100.0, abs(pct))
        return 0.0 if a < 1.0 else self.pwm_min + a * (100.0 - self.pwm_min) / 100.0

    def velocidad(self, pct):
        """Velocidad estimada de una rueda (mm/s) para un comando en %."""
        ref = self._duty(self.vel_base) - self.zm
        d = self._duty(pct) - self.zm
        if d <= 0 or ref <= 0:
            return 0.0
        v = d / ref * self.vel_mm_s
        return v if pct >= 0 else -v

    def actualizar(self, izq, der, dt):
        if dt <= 0 or dt > 0.2:
            return self.rumbo
        a = min(1.0, dt / self.tau) if self.tau > 0 else 1.0
        self.vl += a * (self.velocidad(izq) - self.vl)
        self.vr += a * (self.velocidad(der) - self.vr)
        w = (self.vl - self.vr) / self.b              # rad/s, positivo = giro a la derecha
        self.w_dps = math.degrees(w)
        self.rumbo = (self.rumbo + self.w_dps * dt) % 360.0
        v = (self.vl + self.vr) / 2
        th = math.radians(self.rumbo)
        self.x += v * math.sin(th) * dt
        self.y += v * math.cos(th) * dt
        return self.rumbo

    def calibrar_giro(self, vel_giro, grados_s):
        """Con la velocidad de giro medida (°/s) a vel_giro, ajusta la trocha efectiva."""
        v = self.velocidad(vel_giro)
        if grados_s <= 0 or v <= 0:
            return False
        self.b = 2 * v / math.radians(grados_s)
        self.grados_s = grados_s
        self.calibrada = True
        return True

    def cero(self):
        self.rumbo = 0.0
        self.x = self.y = 0.0

    def a_dict(self):
        return {"b": self.b, "grados_s": self.grados_s}

    def de_dict(self, d):
        if d and d.get("b"):
            self.b = d["b"]
            self.grados_s = d.get("grados_s", 0.0)
            self.calibrada = True
