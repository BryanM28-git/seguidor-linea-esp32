import os, sys, math, json, random
D = os.path.dirname(os.path.abspath(__file__)); os.chdir(D); sys.path.insert(0, D)
if os.path.exists("calib.json"): os.remove("calib.json")
import sim
from sim import R, nuevo_mundo, construir_pista, correr_corrutina, bucle
random.seed(5)
SEGS = [("S", 600), ("A", 180, 250), ("A", 90, 150), ("A", -90, 150), ("A", -90, 150), ("A", 90, 150), ("A", 180, 250)]
P = construir_pista(SEGS); L = len(P) * 2.0
res = {}

def mundo(**k):
    W = nuevo_mundo(**k); W.hay_brujula = False; return W

W = mundo(pista=P, tramos=[], obstaculos=[])
import main as M
rob = M.Robot(); W.robot = rob
res["brujula"] = rob.telemetria()["brujula"]
# robot centrado sobre el tramo recto, mirando a lo largo de la línea
W.x, W.y, W.th = P[150][0], P[150][1], 0.0
correr_corrutina(rob.calibrar_linea(), rob, W)
W.x, W.y, W.th = P[150][0], P[150][1], 0.0
correr_corrutina(rob.calibrar_giro(), rob, W)
v40 = (rob.mot._duty(40) / 655.35 - 22) / 78 * 520
real = math.degrees(2 * v40 / W.B)
res["giro_medido_gps"] = round(rob.odo.grados_s, 1); res["giro_real_gps"] = round(real, 1)
res["giro_error_pct"] = round(100 * (rob.odo.grados_s - real) / real, 1)
for col in ["negro", "rojo", "amarillo", "verde", "azul"]:
    W.tramos = [(0, 1e9, col)]; W.x, W.y, W.th = P[150][0] - 78.5, P[150][1], 0.0; rob.procesar("cal_color:" + col)
W.x, W.y = P[150][0] - 78.5, P[150][1] + 120; rob.procesar("cal_color:blanco")

# giros manuales: compara el giro REAL del mundo con el pedido
W.tramos = []; W.x, W.y, W.th = 0, 300, math.radians(90)
rob.procesar("cero"); err = {}
for cmd, pedido in (("derecha", 90), ("izquierda", -90), ("izquierda", -90), ("girar:45", 45), ("rumbo:0", None)):
    th0 = W.th; ant = rob.rumbo
    rob.procesar(cmd); bucle(rob, W, 3)
    real_deg = -math.degrees(W.th - th0)                  # horario positivo
    if pedido is None: pedido = M.dif_angulo(0, ant)
    err[cmd + "#%d" % len(err)] = round(real_deg - pedido, 1)
res["error_giros_reales_grados"] = err
th0 = W.th; x0, y0 = W.x, W.y; rob.procesar("adelante"); bucle(rob, W, 1.6)
res["adelante_desvio_grados"] = round(abs(math.degrees(W.th - th0)), 1)

tramos = [(60, 170, "rojo"), (900, 1020, "azul"), (1450, 1570, "verde"), (2050, 2160, "amarillo")]
ob = [(P[280][0], P[280][1], 45), (P[1380][0], P[1380][1], 45)]
def auto(con_servo, archivo):
    sim.C.PIN_SERVO = 5 if con_servo else None
    W2 = mundo(pista=P, tramos=tramos, obstaculos=ob)
    r = M.Robot(); W2.robot = r
    r.procesar("cero"); r.procesar("auto")
    th_ini = W2.th
    bucle(r, W2, 60)
    s_vals = [t[3] for t in W2.traza]; prog, prev = 0.0, s_vals[0]
    for s in s_vals[1:]:
        d = s - prev
        if d < -L / 2: d += L
        if abs(d) < 200: prog += d
        prev = s
    real = (-math.degrees(W2.th - th_ini)) % 360
    p = "servo_" if con_servo else "fijo_"
    res[p + "vueltas"] = round(prog / L, 2); res[p + "choques_ticks"] = W2.choques
    res[p + "obstaculos"] = r.obstaculos; res[p + "estado_final"] = r.estado
    res[p + "colores"] = sorted(set(c for _, c in r.historial_colores) - {"blanco", "negro"})
    res[p + "deriva_rumbo_60s"] = round(M.dif_angulo(r.rumbo, real), 1)
    json.dump({"traza": [(round(x), round(y), e) for x, y, e, _ in W2.traza[::5]], "pista": P[::4, :2].round().tolist(),
               "obst": ob, "tramos": tramos}, open(archivo, "w"))
auto(False, "sb_fijo.json"); auto(True, "sb_servo.json")
print(json.dumps(res, ensure_ascii=False, indent=1))
