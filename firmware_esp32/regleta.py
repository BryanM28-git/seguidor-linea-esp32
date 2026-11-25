# regleta.py — Regleta infrarroja de 8 canales tipo RC (QRE1113 / QTR-8RC)
# Lectura RC: se carga el condensador de cada sensor (pin en salida, 1) y se mide cuánto
# tarda en descargarse (pin en entrada). Superficie oscura = más tiempo.
# Los 8 pines se leen a la vez desde los registros GPIO del ESP32 para tener buena resolución.
import time
import machine
import micropython
from machine import Pin

_GPIO_IN = 0x3FF4403C    # GPIO 0..31  (ESP32 clásico)
_GPIO_IN1 = 0x3FF44040   # GPIO 32..39


class RegletaRC:
    def __init__(self, pines, timeout_us=2500, umbral=300, linea_negra=True):
        self.n = len(pines)
        # pull=None quita las resistencias internas: el GPIO4 trae pull-down de fábrica y
        # descargaba el condensador solo (leía ~200 µs siempre, como si viera blanco).
        self.pines = [Pin(p, Pin.IN, pull=None) for p in pines]
        self._reg = [0 if p < 32 else 1 for p in pines]
        self._bit = [1 << (p if p < 32 else p - 32) for p in pines]
        self._m0 = 0
        self._m1 = 0
        for r, b in zip(self._reg, self._bit):
            if r == 0:
                self._m0 |= b
            else:
                self._m1 |= b
        self.timeout = timeout_us
        self.umbral = umbral
        self.linea_negra = linea_negra
        self.crudo = [timeout_us] * self.n
        self.cal_min = [timeout_us] * self.n
        self.cal_max = [0] * self.n
        self.calibrada = False
        self.valores = [0] * self.n
        self.posicion = (self.n - 1) * 500
        self.hay_linea = False

    # -------------------------------------------------------------- lectura
    def leer_crudo(self):
        for p in self.pines:
            p.init(Pin.OUT, value=1)
        time.sleep_us(10)
        for p in self.pines:
            p.init(Pin.IN)
        self._medir()
        return self.crudo

    @micropython.native
    def _medir(self):
        to = self.timeout
        t = self.crudo
        for i in range(self.n):
            t[i] = to
        pend0 = self._m0
        pend1 = self._m1
        reg = self._reg
        bit = self._bit
        mem = machine.mem32
        t0 = time.ticks_us()
        while pend0 | pend1:
            dt = time.ticks_diff(time.ticks_us(), t0)
            if dt > to:
                break
            c0 = pend0 & ~mem[_GPIO_IN]
            c1 = pend1 & ~mem[_GPIO_IN1]
            if c0 | c1:
                for i in range(self.n):
                    if reg[i] == 0:
                        if c0 & bit[i]:
                            t[i] = dt
                    elif c1 & bit[i]:
                        t[i] = dt
                pend0 &= ~c0
                pend1 &= ~c1

    # -------------------------------------------------------------- calibración
    def calibrar_muestra(self):
        """Llamar muchas veces mientras el robot barre la regleta sobre la línea y el fondo."""
        self.leer_crudo()
        for i, v in enumerate(self.crudo):
            if v < self.cal_min[i]:
                self.cal_min[i] = v
            if v > self.cal_max[i]:
                self.cal_max[i] = v

    def reiniciar_calibracion(self):
        self.cal_min = [self.timeout] * self.n
        self.cal_max = [0] * self.n
        self.calibrada = False

    def terminar_calibracion(self):
        # cada sensor debe haber visto línea Y fondo con buen contraste
        ok = all(self.cal_max[i] - self.cal_min[i] > 0.25 * self.timeout for i in range(self.n))
        self.calibrada = ok
        return ok

    # -------------------------------------------------------------- posición
    def actualizar(self):
        """Lee y calcula valores 0..1000 (1000 = línea), posición 0..7000 y si hay línea."""
        self.leer_crudo()
        return self.procesar(self.crudo)

    def procesar(self, crudo):
        n = self.n
        suma = 0
        pond = 0
        maximo = 0
        for i in range(n):
            lo = self.cal_min[i] if self.calibrada else 0
            hi = self.cal_max[i] if self.calibrada else self.timeout
            rango = hi - lo
            v = 0 if rango <= 0 else (crudo[i] - lo) * 1000 // rango
            v = 0 if v < 0 else (1000 if v > 1000 else v)
            if not self.linea_negra:
                v = 1000 - v
            self.valores[i] = v
            if v > maximo:
                maximo = v
            if v > 50:                      # ruido de fondo fuera del promedio
                suma += v
                pond += v * i * 1000
        self.hay_linea = maximo >= self.umbral
        if self.hay_linea and suma > 0:
            self.posicion = pond // suma
        else:
            # sin línea: se queda en el extremo por donde se perdió
            self.posicion = 0 if self.posicion < (n - 1) * 500 else (n - 1) * 1000
        return self.posicion

    def error(self):
        """Error normalizado -1 (línea a la izquierda) .. 1 (línea a la derecha)."""
        centro = (self.n - 1) * 500
        return (self.posicion - centro) / centro

    def a_dict(self):
        return {"min": self.cal_min, "max": self.cal_max}

    def de_dict(self, d):
        if d and len(d.get("min", [])) == self.n:
            self.cal_min = list(d["min"])
            self.cal_max = list(d["max"])
            self.calibrada = True
