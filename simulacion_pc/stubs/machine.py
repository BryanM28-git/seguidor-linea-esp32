# machine simulado: conecta los periféricos con el mundo de simulacion (sim.MUNDO)
import struct
MUNDO = None      # lo asigna sim.py
class Pin:
    IN, OUT = 0, 1
    PULL_UP, PULL_DOWN = 1, 2
    _todos = {}
    def __init__(self, n, mode=0, pull=-1, value=None):
        self.n = n; self.mode = mode; self._v = value or 0
        Pin._todos[n] = self
    def init(self, mode=0, pull=-1, value=None):
        self.mode = mode
        if value is not None: self._v = value
        if mode == Pin.IN and MUNDO: MUNDO.pin_a_entrada(self.n)
    def value(self, v=None):
        if v is None: return self._v
        self._v = v
    def __call__(self, v=None): return self.value(v)
class PWM:
    def __init__(self, pin, freq=1000, duty_u16=0): self.pin = pin; self.d = duty_u16
    def duty_u16(self, d=None):
        if d is None: return self.d
        self.d = d
class ADC:
    ATTN_11DB = 3
    def __init__(self, pin): pass
    def atten(self, a): pass
    def read_uv(self): return 2_590_000
class I2C:
    def __init__(self, *a, **k): pass
    def scan(self): return [0x0D] if getattr(MUNDO, 'hay_brujula', True) else []
    def writeto_mem(self, a, r, b): pass
    def readfrom_mem(self, a, r, n):
        x, y, z = MUNDO.magnetometro()
        return struct.pack("<hhh", x, y, z)
class _Mem:
    def __getitem__(self, addr): return MUNDO.registro_gpio(addr)
mem32 = _Mem()
def time_pulse_us(pin, nivel, timeout):
    return MUNDO.pulso(pin.n, nivel, timeout)
