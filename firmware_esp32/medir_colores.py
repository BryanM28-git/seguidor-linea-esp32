# medir_colores.py — ¿La regleta IR ve cada color de la pista? (SIN WiFi ni motores)
# En Thonny: pulsa STOP, abre este archivo y pulsa F5. Sigue las instrucciones de la consola.
import time
import config as C
from regleta import RegletaRC
from color import TCS3200

pines = tuple(reversed(C.PIN_QRE)) if C.INVERTIR_REGLETA else C.PIN_QRE
reg = RegletaRC(pines, C.RC_TIMEOUT_US)
tcs = TCS3200(C.PIN_S2, C.PIN_S3, C.PIN_TCS_OUT)
COLORES = ("blanco", "negro", "rojo", "morado", "amarillo", "verde", "azul")


def promedio_regleta(n=20):
    acc = [0] * 8
    for _ in range(n):
        for i, v in enumerate(reg.leer_crudo()):
            acc[i] += v
        time.sleep_ms(10)
    return [a // n for a in acc]


def promedio_color(n=4):
    acc = [0.0] * 4
    for _ in range(n):
        for i, v in enumerate(tcs.leer()):
            acc[i] += v
    return [a / n for a in acc]


print("\nPon el robot en la pista. Para cada color, centra la regleta (sensores 4 y 5)")
print("y el sensor de color SOBRE ese color, y pulsa Enter. 'blanco' = todo sobre papel blanco.")
print("Si ese color NO está en la pista, escribe  n  y pulsa Enter para saltarlo.\n")
datos = {}
for col in COLORES:
    if col == "blanco":
        input(">> Coloca el robot sobre BLANCO (obligatorio) y pulsa Enter: ")
    else:
        r = input(">> Coloca el robot sobre %s y pulsa Enter  (n + Enter = no hay): " % col.upper())
        if r.strip().lower() in ("n", "no", "s", "saltar"):
            print("   (saltado)")
            continue
    reg_v = promedio_regleta()
    rgbc = promedio_color()
    datos[col] = (reg_v, rgbc)
    print("   regleta:", " ".join("%4d" % v for v in reg_v),
          "| R %5.0f G %5.0f B %5.0f C %5.0f" % tuple(rgbc))

blanco = max(datos["blanco"][0])
print("\n================ RESUMEN (mándale una foto a Claude) ================")
print("color      máx regleta  contraste   ¿IR la ve?     r     g     b   brillo")
for col in COLORES:
    if col not in datos:
        continue
    reg_v, (r, g, b, c) = datos[col]
    m = max(reg_v)
    k = m - blanco
    if k > 300:
        v = "SÍ"
    elif k > 40:
        v = "débil"
    else:
        v = "NO"
    s = r + g + b or 1
    cb = datos["blanco"][1][3] or 1
    print("%-9s  %8d  %9d     %-6s   %.2f  %.2f  %.2f  %.2f" % (col, m, k, v, r / s, g / s, b / s, c / cb))
print("Luz ambiente: repite con el robot a la sombra si 'blanco' da valores muy bajos y nada contrasta.")
