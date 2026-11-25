# motores.py — L298N controlado con PWM en IN1..IN4 (jumpers ENA/ENB puestos)
from machine import Pin, PWM


class Motores:
    def __init__(self, in1, in2, in3, in4, freq=1000, pwm_min=28, inv_izq=False, inv_der=False, ajuste=0.0):
        self._p = [PWM(Pin(n), freq=freq, duty_u16=0) for n in (in1, in2, in3, in4)]
        self.pwm_min = pwm_min
        self.inv = (-1 if inv_izq else 1, -1 if inv_der else 1)
        self.izq = 0.0
        self.der = 0.0
        # ajuste (%) para que vayan parejas: negativo frena la IZQUIERDA, positivo frena la DERECHA.
        # Es proporcional, así sirve a cualquier velocidad, adelante, atrás y en los giros.
        self.ajuste = ajuste

    def _duty(self, v):
        """v en % (-100..100) -> duty_u16 con compensación de zona muerta."""
        a = min(100.0, abs(v))
        if a < 1.0:
            return 0
        a = self.pwm_min + a * (100.0 - self.pwm_min) / 100.0
        return int(a * 655.35)

    def _lado(self, pa, pb, v):
        d = self._duty(v)
        if v >= 0:
            pb.duty_u16(0)
            pa.duty_u16(d)
        else:
            pa.duty_u16(0)
            pb.duty_u16(d)

    def mover(self, izq, der):
        """Velocidades en % de -100 a 100 (positivo = adelante)."""
        self.izq = max(-100.0, min(100.0, izq))
        self.der = max(-100.0, min(100.0, der))
        k = self.ajuste / 100.0
        fi = 1.0 + k if k < 0 else 1.0        # solo se frena la rueda rápida, nunca se pasa de 100 %
        fd = 1.0 - k if k > 0 else 1.0
        self._lado(self._p[0], self._p[1], self.izq * fi * self.inv[0])
        self._lado(self._p[2], self._p[3], self.der * fd * self.inv[1])

    def parar(self):
        self.mover(0, 0)
