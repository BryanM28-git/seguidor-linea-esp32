# color.py — TCS3200 / GY-31: lectura de frecuencias y clasificación de 6 colores
# S0=3V3 y S1=GND (escala 20 %). S2/S3 eligen el filtro. OUT es una onda cuadrada (50 %).
import time
import math
from machine import Pin, time_pulse_us

_FILTRO = {"r": (0, 0), "g": (1, 1), "b": (0, 1), "c": (1, 0)}


class TCS3200:
    def __init__(self, s2, s3, out, ciclos=3):
        self.s2 = Pin(s2, Pin.OUT, value=0)
        self.s3 = Pin(s3, Pin.OUT, value=0)
        self.out = Pin(out, Pin.IN)
        self.ciclos = ciclos
        self.ref = {}            # nombre -> [r, g, b, c] en Hz (calibrado)
        self.ultimo = [0, 0, 0, 0]

    def frecuencia(self, f):
        a, b = _FILTRO[f]
        self.s2.value(a)
        self.s3.value(b)
        time.sleep_us(120)                     # el filtro tarda en estabilizarse
        total = 0
        for _ in range(self.ciclos):
            t = time_pulse_us(self.out, 1, 4000)  # semiperiodo en alto
            if t <= 0:
                return 0.0                     # muy oscuro o sin señal
            total += t
        return 500000.0 * self.ciclos / total  # Hz

    def leer(self):
        self.ultimo = [self.frecuencia(k) for k in ("r", "g", "b", "c")]
        return self.ultimo

    def leer_paso(self):
        """Lee UN filtro por llamada (bloquea ~1-3 ms). Devuelve True cuando completó R, G, B y C."""
        if not hasattr(self, "_paso"):
            self._paso, self._tmp = 0, [0.0] * 4
        k = ("r", "g", "b", "c")[self._paso]
        self._tmp[self._paso] = self.frecuencia(k)
        self._paso = (self._paso + 1) % 4
        if self._paso == 0:
            self.ultimo = list(self._tmp)
            return True
        return False

    # -------------------------------------------------------------- clasificación
    @staticmethod
    def rasgos(rgbc, c_blanco):
        r, g, b, c = rgbc
        s = r + g + b
        if s <= 0:
            return (1 / 3, 1 / 3, 1 / 3, 0.0)
        brillo = c / c_blanco if c_blanco > 0 else 0.0
        return (r / s, g / s, b / s, min(brillo, 1.5))

    def _c_blanco(self):
        w = self.ref.get("blanco")
        if w and w[3] > 0:
            return w[3]
        return max([v[3] for v in self.ref.values()] + [1.0])

    def clasificar(self, rgbc=None, dist_max=0.35):
        """Devuelve (nombre, distancia). Usa cromaticidad (r,g,b normalizados) + brillo."""
        if not self.ref:
            return ("sin calibrar", 0.0)
        rgbc = rgbc or self.ultimo
        cb = self._c_blanco()
        x = self.rasgos(rgbc, cb)
        mejor, dmin = "desconocido", 1e9
        for nombre, ref in self.ref.items():
            y = self.rasgos(ref, cb)
            d = math.sqrt(4 * ((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2 + (x[2] - y[2]) ** 2)
                          + (x[3] - y[3]) ** 2)
            if d < dmin:
                mejor, dmin = nombre, d
        if dmin > dist_max:
            return ("desconocido", dmin)
        return (mejor, dmin)

    def calibrar(self, nombre, n=12):
        acc = [0.0, 0.0, 0.0, 0.0]
        for _ in range(n):
            v = self.leer()
            for i in range(4):
                acc[i] += v[i]
            time.sleep_ms(15)
        self.ref[nombre] = [round(a / n, 1) for a in acc]
        return self.ref[nombre]

    def rgb_aprox(self):
        """RGB 0..255 aproximado para mostrar en la app (escala con el blanco calibrado)."""
        w = self.ref.get("blanco")
        k = self.ref.get("negro")
        out = []
        for i in range(3):
            v = self.ultimo[i]
            lo = k[i] if k else 0.0
            hi = w[i] if w else max(self.ultimo[:3] + [1.0])
            out.append(int(max(0, min(255, (v - lo) * 255 / max(hi - lo, 1e-6)))))
        return out
