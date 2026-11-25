# brujula.py — GY-271 (QMC5883L en 0x0D o HMC5883L en 0x1E), con calibración hard/soft-iron simple
import math
import struct


class Brujula:
    def __init__(self, i2c, declinacion=0.0, invertir=False):
        self.i2c = i2c
        self.decl = declinacion
        self.inv = invertir
        self.off = [0.0, 0.0]
        self.esc = [1.0, 1.0]
        self.calibrada = False
        self.rumbo = 0.0
        self._s = 0.0
        self._c = 1.0
        disp = i2c.scan()
        if 0x0D in disp:
            self.tipo, self.addr = "QMC5883L", 0x0D
            i2c.writeto_mem(0x0D, 0x0B, b"\x01")      # periodo SET/RESET
            i2c.writeto_mem(0x0D, 0x09, b"\x1D")      # continuo, 200 Hz, 8 G, OSR 512
        elif 0x1E in disp:
            self.tipo, self.addr = "HMC5883L", 0x1E
            i2c.writeto_mem(0x1E, 0x00, b"\x78")      # 8 promedios, 75 Hz
            i2c.writeto_mem(0x1E, 0x01, b"\x20")      # ganancia 1,3 G
            i2c.writeto_mem(0x1E, 0x02, b"\x00")      # continuo
        else:
            raise OSError("Brújula no encontrada. Dispositivos I2C: %s" % [hex(d) for d in disp])

    def crudo(self):
        if self.tipo == "QMC5883L":
            x, y, z = struct.unpack("<hhh", self.i2c.readfrom_mem(self.addr, 0x00, 6))
        else:
            x, z, y = struct.unpack(">hhh", self.i2c.readfrom_mem(self.addr, 0x03, 6))
        return x, y, z

    @staticmethod
    def calcular_rumbo(x, y, off, esc, decl=0.0, inv=False):
        """Rumbo 0..360 (horario desde el norte) con X del sensor hacia adelante e Y a la izquierda."""
        xc = (x - off[0]) * esc[0]
        yc = (y - off[1]) * esc[1]
        h = math.degrees(math.atan2(yc, xc))
        if inv:
            h = -h
        return (h + decl) % 360.0

    def leer(self, alfa=0.6):
        x, y, _ = self.crudo()
        h = self.calcular_rumbo(x, y, self.off, self.esc, self.decl, self.inv)
        # filtro paso bajo circular (evita el salto 359 -> 0)
        r = math.radians(h)
        self._s += alfa * (math.sin(r) - self._s)
        self._c += alfa * (math.cos(r) - self._c)
        self.rumbo = math.degrees(math.atan2(self._s, self._c)) % 360.0
        return self.rumbo

    # -------------------------------------------------------------- calibración
    def iniciar_calibracion(self):
        self._mn = [1e9, 1e9]
        self._mx = [-1e9, -1e9]

    def muestra_calibracion(self):
        x, y, _ = self.crudo()
        for i, v in enumerate((x, y)):
            self._mn[i] = min(self._mn[i], v)
            self._mx[i] = max(self._mx[i], v)

    def terminar_calibracion(self):
        rx = self._mx[0] - self._mn[0]
        ry = self._mx[1] - self._mn[1]
        if rx < 50 or ry < 50:
            return False
        self.off = [(self._mx[0] + self._mn[0]) / 2, (self._mx[1] + self._mn[1]) / 2]
        prom = (rx + ry) / 2
        self.esc = [prom / rx, prom / ry]
        self.calibrada = True
        return True

    def a_dict(self):
        return {"off": self.off, "esc": self.esc}

    def de_dict(self, d):
        if d and "off" in d:
            self.off = list(d["off"])
            self.esc = list(d["esc"])
            self.calibrada = True


def dif_angulo(objetivo, actual):
    """Diferencia firmada en grados (-180..180). Positivo = girar a la derecha."""
    return (objetivo - actual + 540.0) % 360.0 - 180.0
