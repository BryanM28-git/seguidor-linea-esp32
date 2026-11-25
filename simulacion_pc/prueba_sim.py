import os, sys, math, json, random, asyncio, urllib.request, threading, time as T
os.chdir(os.path.dirname(os.path.abspath(__file__)))
if os.path.exists("calib.json"): os.remove("calib.json")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim
from sim import R, nuevo_mundo, construir_pista, correr_corrutina, bucle
random.seed(3)

SEGS = [("S", 600), ("A", 180, 250), ("A", 90, 150), ("A", -90, 150), ("A", -90, 150), ("A", 90, 150), ("A", 180, 250)]
P = construir_pista(SEGS)
L = len(P) * 2.0
print("pista: %.0f mm" % L)
res = {}

# ------------------------------------------------------------ 1. robot + calibraciones
W = nuevo_mundo(pista=P, tramos=[], obstaculos=[])
import main as M
rob = M.Robot()
print("brújula:", rob.bru.tipo if rob.bru else rob.error_brujula)

# regleta RC por registros: compara con los tiempos del mundo
rob.reg.leer_crudo()
err_rc = max(abs(a - b) for a, b in zip(rob.reg.crudo, W.rc_t))
res["error_lectura_rc_us"] = round(err_rc, 1)

correr_corrutina(rob.calibrar_linea(), rob, W)
res["regleta_calibrada"] = rob.reg.calibrada
correr_corrutina(rob.calibrar_brujula(), rob, W)
res["brujula_calibrada"] = rob.bru.calibrada
errs = []
for th in range(0, 360, 30):
    W.th = math.radians(th)
    for _ in range(12): rob.bru.leer()
    errs.append(abs(M.dif_angulo(rob.bru.leer(), (90 - th) % 360)))
res["error_brujula_max_grados"] = round(max(errs), 1)

# colores: pone el TCS sobre cada color
W.th = 0.0
for col in ["negro", "rojo", "amarillo", "verde", "azul"]:
    W.tramos = [(0, 1e9, col)]
    W.x, W.y = P[150][0] - 78.5, P[150][1]
    rob.procesar("cal_color:" + col)
W.x, W.y = P[150][0] - 78.5, P[150][1] + 120          # fuera de la línea = blanco
rob.procesar("cal_color:blanco")
aciertos = 0
for col in ["negro", "rojo", "amarillo", "verde", "azul", "blanco"]:
    W.tramos = [(0, 1e9, col)]
    W.x, W.y = P[150][0] - 78.5, P[150][1] + (120 if col == "blanco" else 0)
    ok = 0
    for _ in range(20):
        rob.tcs.leer(); ok += rob.tcs.clasificar(dist_max=0.35)[0] == col
    aciertos += ok
res["acierto_colores_pct"] = round(aciertos / 120 * 100, 1)

# ------------------------------------------------------------ 2. modo manual (brújula)
W.tramos = []; W.x, W.y, W.th = 0, 300, math.radians(90)
for _ in range(15): rob.rumbo = rob.bru.leer()
ini = rob.rumbo
for cmd, esperado in (("derecha", 90), ("izquierda", 0), ("rumbo:%d" % ((ini + 180) % 360), 180)):
    rob.procesar(cmd)
    bucle(rob, W, 4)
    res["manual_" + cmd.split(":")[0]] = round(M.dif_angulo(rob.rumbo, (ini + esperado) % 360), 1)
th0, x0, y0 = W.th, W.x, W.y
rob.procesar("adelante"); bucle(rob, W, 1.6)
avance = math.hypot(W.x - x0, W.y - y0)
desvio = abs(math.degrees(W.th - th0))
res["manual_adelante_mm"] = round(avance); res["manual_adelante_desvio_grados"] = round(desvio, 1)

# ------------------------------------------------------------ 3. modo autónomo: 60 s en la pista
tramos = [(60, 170, "rojo"), (900, 1020, "azul"), (1450, 1570, "verde"), (2050, 2160, "amarillo")]
ob = [(P[280][0], P[280][1], 45), (P[1380][0], P[1380][1], 45)]

def correr_auto(con_servo, archivo):
    sim.C.PIN_SERVO = 5 if con_servo else None
    W2 = nuevo_mundo(pista=P, tramos=tramos, obstaculos=ob)
    r = M.Robot()                                 # carga calib.json guardado arriba
    W2.robot = r
    for _ in range(15): r.rumbo = r.bru.leer()
    r.procesar("auto")
    desv, estados = [], set()
    def reg(k, rr, w):
        estados.add(rr.estado)
        if rr.estado == "SEGUIR": desv.append(w.cercano(*w.a_mundo(107, 0))[0])
    bucle(r, W2, 60, reg)
    s_vals = [t[3] for t in W2.traza]
    progreso, prev = 0.0, s_vals[0]
    for s in s_vals[1:]:
        d = s - prev
        if d < -L / 2: d += L
        if abs(d) < 200: progreso += d
        prev = s
    pref = "servo_" if con_servo else "fijo_"
    res[pref + "vueltas"] = round(progreso / L, 2)
    res[pref + "desvio_regleta_mm_p95"] = round(sorted(desv)[int(len(desv) * .95)], 1)
    res[pref + "obstaculos_esquivados"] = r.obstaculos
    res[pref + "ticks_en_choque"] = W2.choques
    res[pref + "colores"] = [c for _, c in r.historial_colores if c != "blanco"][:12]
    res[pref + "estado_final"] = r.estado
    res[pref + "eventos"] = [e for _, e in r.eventos][-6:]
    json.dump({"traza": [(round(x), round(y), e) for x, y, e, _ in W2.traza[::5]], "pista": P[::4, :2].round().tolist(),
               "obst": ob, "tramos": tramos}, open(archivo, "w"))
    return r

correr_auto(False, "sim_fijo.json")
rob = correr_auto(True, "sim_servo.json")

# ------------------------------------------------------------ 4. servidor HTTP real (asyncio de CPython)
async def servidor():
    srv = await asyncio.start_server(rob.manejar, "127.0.0.1", 8765)
    async with srv: await srv.serve_forever()
th = threading.Thread(target=lambda: asyncio.run(servidor()), daemon=True); th.start(); T.sleep(0.5)
est = json.loads(urllib.request.urlopen("http://127.0.0.1:8765/estado", timeout=3).read())
cmd = json.loads(urllib.request.urlopen("http://127.0.0.1:8765/cmd?c=rumbo%3A45", timeout=3).read())
res["http_estado_campos"] = len(est); res["http_cmd"] = cmd
print(json.dumps(res, ensure_ascii=False, indent=1))
