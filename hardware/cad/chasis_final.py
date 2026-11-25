"""
Chasis FINAL — Robot seguidor de línea (rueda loca adelante, configuración tipo "triciclo")
==========================================================================================
Bryan A. Martínez M. — Ing. Mecatrónica UMNG
Requiere: numpy, scikit-image   (pip install numpy scikit-image)
Uso:      python chasis_final.py [carpeta_salida]

Coordenadas (mm): X adelante, Y izquierda, Z arriba. Origen en el piso bajo el
punto medio del eje de las ruedas motrices.

Materiales (lista de compra Mactronica + Ferretrónica):
  2x motorreductor TT 1:48 ..... 70 x 22.3 x 18.8, rueda Ø65 x 26
  Puente H L298N ............... 43 x 43 x 27, agujeros a 37 mm
  ESP32 30 pines + expansión ... placa 65 x 55, ~25 mm de alto con la DevKit
  HC-SR05 ...................... 49 x 22 x 15, transductores Ø16
  Regleta QRE1113 x8 ........... 75 x 12.7 x 3.2, óptimo 3 mm al piso
  TCS3200 (GY-31) .............. 33 x 33, óptimo 10 mm al piso
  Rodachín 1" (rueda loca) ..... rueda Ø25.4, ALTURA ESTIMADA 34 mm  <-- MEDIR
  Portapilas 2x18650 ........... 77 x 41 x 21 (no está en la lista; recomendado)

Piezas que genera (todas sin soportes):
  final_chasis.stl      -> imprimir BOCA ABAJO (cara superior de la placa sobre la cama)
  final_carcasa.stl     -> imprimir BOCA ABAJO (techo sobre la cama)
  final_calza_rueda.stl -> plana
  final_separadores.stl -> 8 separadores M3 de 6 mm
"""
import json, os, struct, sys
import numpy as np
from skimage.measure import marching_cubes

# ============================================================ PARÁMETROS (mm)
R_RUEDA, ANCHO_RUEDA = 32.5, 26.0
MOTOR_L, MOTOR_W, MOTOR_H = 70.0, 22.3, 18.8
EJE_A_PUNTA = 11.5            # eje -> extremo de la caja reductora
Y_INT = 26.0                  # cara interior del motor (deja 2 mm para la brida)
Z_EJE = R_RUEDA
Z_MOT0, Z_MOT1 = Z_EJE - MOTOR_H / 2, Z_EJE + MOTOR_H / 2
T = 3.0                       # espesor de la placa
Z_P0 = Z_MOT1 + 0.3           # cara inferior de la placa (motores colgados) = 42.2
Z_P1 = Z_P0 + T               # cara superior de la placa = 45.2

CASTER_H   = float(os.environ.get("CASTER_H", 34.0))  # piso -> cara de montaje del rodachín  <-- MEDIR
CASTER_X   = 142.0            # pivote de la rueda loca (ADELANTE de los sensores)
CASTER_HX, CASTER_HY = 26.0, 20.0     # distancia entre agujeros de la platina  <-- MEDIR
CASTER_PL  = (38.0, 32.0)     # platina del rodachín
SWIVEL_R   = 23.0             # radio barrido por la horquilla al girar

QRE_H, QRE_E, QRE_A, QRE_L = 3.0, 3.2, 12.7, 75.0
QRE_HOLE_Y = 34.5             # semidistancia entre agujeros M2 de la regleta
TCS_X, TCS_H, TCS_LADO, TCS_LED = 78.0, 10.0, 33.0, 5.0
QRE_X = TCS_X + 19 + 2.6 + QRE_A / 2  # = 105.95
POD_X0, POD_X1, POD_Y = 59.0, 117.0, 42.0   # faldón anti-luz
HC_PASO = 27.0                # centro a centro de transductores (ranura admite 24..30)
L298_X, ESP_X = 25.0, 112.0
X_REAR, X_FRONT, Y_HALF = -40.0, 167.0, 46.0
Z_TOP = 86.0
H_GRID = float(os.environ.get("H_GRID", 0.6))

# ============================================================ SDF
def box(G, c, s, r=0.0):
    X, Y, Z = G
    qx = np.abs(X - c[0]) - s[0] / 2 + r
    qy = np.abs(Y - c[1]) - s[1] / 2 + r
    qz = np.abs(Z - c[2]) - s[2] / 2 + r
    out = np.sqrt(np.maximum(qx, 0)**2 + np.maximum(qy, 0)**2 + np.maximum(qz, 0)**2)
    return out + np.minimum(np.maximum(qx, np.maximum(qy, qz)), 0) - r

def bx(G, x0, x1, y0, y1, z0, z1, r=0.0):
    return box(G, ((x0+x1)/2, (y0+y1)/2, (z0+z1)/2), (x1-x0, y1-y0, z1-z0), r)

def cyl(G, c, rad, h, ax="z"):
    X, Y, Z = G
    if ax == "z":   rr, aa = np.hypot(X-c[0], Y-c[1]), Z-c[2]
    elif ax == "y": rr, aa = np.hypot(X-c[0], Z-c[2]), Y-c[1]
    else:           rr, aa = np.hypot(Y-c[1], Z-c[2]), X-c[0]
    dr, da = rr - rad, np.abs(aa) - h/2
    return np.minimum(np.maximum(dr, da), 0) + np.hypot(np.maximum(dr, 0), np.maximum(da, 0))

def rrect_prism(G, x0, x1, y0, y1, z0, z1, r):
    """Prisma con esquinas redondeadas sólo en planta (aristas verticales)."""
    X, Y, Z = G
    cx, cy, hx, hy = (x0+x1)/2, (y0+y1)/2, (x1-x0)/2, (y1-y0)/2
    qx, qy = np.abs(X-cx) - hx + r, np.abs(Y-cy) - hy + r
    d2 = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0) - r
    dz = np.abs(Z - (z0+z1)/2) - (z1-z0)/2
    return np.minimum(np.maximum(d2, dz), 0) + np.hypot(np.maximum(d2, 0), np.maximum(dz, 0))

def plano(G, n, p0):
    n = np.asarray(n, float); n /= np.linalg.norm(n)
    return (G[0]-p0[0])*n[0] + (G[1]-p0[1])*n[1] + (G[2]-p0[2])*n[2]

def stadium_x(G, xc, yc, zc, r, half, depth):
    """Agujero ovalado (alargado en Y) que atraviesa una pared normal a X."""
    X, Y, Z = G
    dy = np.maximum(np.abs(Y - yc) - half, 0)
    d2 = np.hypot(dy, Z - zc) - r
    dx = np.abs(X - xc) - depth/2
    return np.minimum(np.maximum(d2, dx), 0) + np.hypot(np.maximum(d2, 0), np.maximum(dx, 0))

U = lambda *d: np.minimum.reduce(d)
S = lambda a, *b: np.maximum(a, -np.minimum.reduce(b))
I = lambda *d: np.maximum.reduce(d)
sym = lambda G: (G[0], np.abs(G[1]), G[2])

# ============================================================ PIEZA 1: CHASIS
def contorno_placa(G, z0, z1):
    return U(rrect_prism(G, X_REAR, X_FRONT, -Y_HALF, Y_HALF, z0, z1, 12),
             rrect_prism(G, 35, 66, -51, 51, z0, z1, 4))          # lóbulos para bridas de motor

def agujeros_tornillos():
    pts = {
        "L298N":   [(L298_X + sx*18.5, sy*18.5) for sx in (-1, 1) for sy in (-1, 1)],
        "ESP32":   [(ESP_X + sx*29.0, sy*24.0) for sx in (-1, 1) for sy in (-1, 1)],
        "carcasa": [(-28, 36), (-28, -36), (153, 36), (153, -36)],
        "rueda":   [(CASTER_X + sx*CASTER_HX/2, sy*CASTER_HY/2) for sx in (-1, 1) for sy in (-1, 1)],
    }
    return pts

def chasis(G):
    Gs = sym(G)
    zc = (Z_P0 + Z_P1) / 2
    d = contorno_placa(G, Z_P0, Z_P1)
    # --- cunas de motor (motor colgado, cuerpo hacia adelante)
    xm0, xm1 = -EJE_A_PUNTA, MOTOR_L - EJE_A_PUNTA          # -11.5 .. 58.5
    d = U(d,
          bx(Gs, xm0-2.5, xm1+2.5, Y_INT-4.4, Y_INT-2.0, Z_EJE-4, Z_P0+0.5),     # pared interior
          bx(Gs, xm0-3.0, xm0-0.3, Y_INT, Y_INT+MOTOR_W, 30, Z_P0+0.5),         # tope trasero
          bx(Gs, xm1+0.3, xm1+2.8, Y_INT, Y_INT+MOTOR_W, 30, Z_P0+0.5))         # tope delantero
    # --- módulo de sensores: faldón anti-luz
    d = U(d, S(bx(G, POD_X0, POD_X1, -POD_Y, POD_Y, 2.0, Z_P0+0.5, 3),
               bx(G, POD_X0+2, POD_X1-2, -POD_Y+2, POD_Y-2, 1.0, Z_P0, 2)))
    # --- túnel del TCS3200 (paredes + embudo a 45° donde se apoya la PCB a z=15)
    o = TCS_LADO + 5
    tun = S(bx(G, TCS_X-o/2, TCS_X+o/2, -o/2, o/2, 2.0, Z_P0+0.5),
            bx(G, TCS_X-17, TCS_X+17, -17, 17, 15.5, Z_P0+1),
            bx(G, TCS_X-14, TCS_X+14, -14, 14, 0, 12.6))
    X, Y, Z = G
    emb = np.maximum(np.abs(X-TCS_X), np.abs(Y)) - (14 + (Z - 12.5))            # embudo 45°
    emb = I(emb, 12.4 - Z, Z - 15.6)
    d = U(d, S(tun, emb))
    # --- postes de la regleta QRE (sin puentes: imprimen verticales)
    zq = QRE_H + QRE_E                                                         # 6.2
    d = U(d, bx(Gs, QRE_X-3, QRE_X+3, QRE_HOLE_Y-4.5, QRE_HOLE_Y+4.5, zq, Z_P0+0.5, 1))
    # --- vaciados
    P = agujeros_tornillos()
    subs = [cyl(G, (x, y, zc), 1.7, 10) for k in ("L298N", "ESP32", "carcasa", "rueda") for x, y in P[k]]
    subs += [bx(Gs, QRE_X-0.8, QRE_X+0.8, QRE_HOLE_Y-2.0, QRE_HOLE_Y+2.0, zq-1, zq+8)]       # ranura M2 (autorroscante)
    subs += [bx(G, TCS_X-17.2, TCS_X+17.2, -17.2, 17.2, Z_P0-1, Z_P1+1, 1)]                  # paso del TCS desde arriba
    subs += [bx(G, QRE_X-8.5, QRE_X-3.5, -24, 24, Z_P0-1, Z_P1+1, 2)]                        # cables de la regleta
    subs += [bx(Gs, 18, 22, Y_INT-1.8, Y_INT-0.2, Z_P0-1, Z_P1+1, 0.7),                      # bridas motor (interior)
             bx(Gs, 40, 44, Y_INT-1.8, Y_INT-0.2, Z_P0-1, Z_P1+1, 0.7),
             bx(Gs, 40, 44, 48.7, 50.2, Z_P0-1, Z_P1+1, 0.7)]                                # bridas motor (lóbulo)
    subs += [bx(Gs, -24, -20, 19.0, 20.6, Z_P0-1, Z_P1+1, 0.7),                              # bridas portapilas
             bx(Gs, 28, 32, 19.0, 20.6, Z_P0-1, Z_P1+1, 0.7)]
    subs += [cyl(Gs, (62.5, 33, zc), 4.5, 10),                                               # cables motores
             bx(G, -34, -26, -8, 8, Z_P0-1, Z_P1+1, 3),                                       # cables batería
             bx(G, 70, 88, 24, 36, Z_P0-1, Z_P1+1, 3)]                                        # aligeramiento/cable TCS
    return S(d, *subs)

# ============================================================ PIEZA 2: CARCASA
def carcasa_ext(G, e=0.0):
    """Volumen exterior (e>0 lo encoge para hacer el hueco)."""
    d = rrect_prism(G, X_REAR+e, X_FRONT-e, -Y_HALF+e, Y_HALF-e, 30, Z_TOP-e, 12-e)
    d = I(d,
          plano(G, (1, 0, 1), (X_FRONT-e, 0, Z_TOP-e-12)),            # chaflán frontal 45° (12 mm)
          plano(G, (-1, 0, 1), (X_REAR+e, 0, Z_TOP-e-8)),            # chaflán trasero 45°
          plano(sym(G), (0, 1, 1), (0, Y_HALF-e, Z_TOP-e-6)))        # chaflanes laterales 45°
    return d

def carcasa(G):
    Gs = sym(G)
    zb = Z_P1 + 0.1
    d = I(S(carcasa_ext(G), carcasa_ext(G, 2.0)), zb - G[2])
    # columnas de fijación
    d = U(d, I(U(*[cyl(G, (x, y, 66), 3.6, 42) for x, y in agujeros_tornillos()["carcasa"]]), zb - G[2], carcasa_ext(G, 1.0)))
    subs = [cyl(G, (x, y, zb+6), 1.3, 12.2) for x, y in agujeros_tornillos()["carcasa"]]
    zhc = Z_P1 + 0.3 + 11.0
    subs += [stadium_x(G, X_FRONT-1, s*HC_PASO/2, zhc, 8.6, 1.8, 6) for s in (-1, 1)]      # "ojos" HC-SR05
    subs += [bx(G, X_REAR-2, X_REAR+4, -9.8, 9.8, 58, 71, 1)]                              # interruptor basculante 19.5x13
    subs += [bx(G, 6+i*7, 9+i*7, -16, 16, Z_TOP-4, Z_TOP+2, 1.4) for i in range(6)]       # ventilación L298N
    subs += [bx(G, 96, 126, Y_HALF-4, Y_HALF+2, 52, 63, 2)]                                # ventana USB ESP32
    subs += [bx(G, 34, 40, -50, 50, zb-1, zb+6, 1.5)]                                      # paso de bridas de motor
    # línea decorativa (bajo relieve 0.8 mm) que recorre el techo
    subs += [bx(G, 60, 150, -1.2, 1.2, Z_TOP-0.8, Z_TOP+1, 1.1)]
    return S(d, *subs)

# ============================================================ PIEZA 3 y 4
def calza_rueda(G):
    """Calza entre placa y rodachín. Espesor = Z_P0 - CASTER_H."""
    t = Z_P0 - CASTER_H
    d = rrect_prism(G, -CASTER_PL[0]/2-1, CASTER_PL[0]/2+1, -CASTER_PL[1]/2-1, CASTER_PL[1]/2+1, 0, t, 5)
    subs = [cyl(G, (sx*CASTER_HX/2, sy*CASTER_HY/2, t/2), 1.7, t+2) for sx in (-1, 1) for sy in (-1, 1)]
    subs += [cyl(G, (0, 0, t/2), 7, t+2)]       # alivio para el remache central del pivote
    return S(d, *subs)

def separadores(G):
    pts = [(x*12, y*12) for x in range(4) for y in range(2)]
    d = U(*[cyl(G, (x, y, 3), 3.2, 6) for x, y in pts])
    return S(d, *[cyl(G, (x, y, 3), 1.35, 8) for x, y in pts])

# ============================================================ ENSAMBLE (componentes)
def componentes():
    ym = Y_INT + MOTOR_W/2
    yw = Y_INT + MOTOR_W + 3 + ANCHO_RUEDA/2
    xb0 = -36.5      # portapilas centrado casi sobre el eje (más carga en ruedas motrices)
    C = [  # nombre, centro, tamaño, color, masa_g
        ("Motorreductor TT", (MOTOR_L/2 - EJE_A_PUNTA, ym, Z_EJE), (MOTOR_L, MOTOR_W, MOTOR_H), "#f1c40f", 30.6),
        ("Motorreductor TT", (MOTOR_L/2 - EJE_A_PUNTA, -ym, Z_EJE), (MOTOR_L, MOTOR_W, MOTOR_H), "#f1c40f", 30.6),
        ("Portapilas 2x18650", (xb0 + 38.5, 0, Z_P0 - 10.6), (77, 41, 21), "#2d2d2d", 107),
        ("Regleta QRE1113 x8", (QRE_X, 0, QRE_H + QRE_E/2), (QRE_A, QRE_L, QRE_E), "#1e8449", 4),
        ("TCS3200 PCB", (TCS_X, 0, 15.0 + 0.8), (TCS_LADO, TCS_LADO, 1.6), "#e67e22", 3),
        ("TCS3200 LEDs", (TCS_X, 0, TCS_H + TCS_LED/2), (20, 20, TCS_LED), "#f5b041", 2),
        ("L298N", (L298_X, 0, Z_P1 + 6 + 13.5), (43, 43, 27), "#c0392b", 30),
        ("ESP32 + expansión", (ESP_X, 0, Z_P1 + 6 + 12.5), (65, 55, 25), "#1f4e9c", 30),
        ("HC-SR05", (X_FRONT - 2 - 7.5 - 0.5, 0, Z_P1 + 0.3 + 11), (15, 49, 22), "#2b7bb9", 10),
        ("Rodachín (platina)", (CASTER_X, 0, CASTER_H - 1), (CASTER_PL[0], CASTER_PL[1], 2), "#95a5a6", 8),
        ("Rodachín (rueda)", (CASTER_X - 8, 0, 12.7), (25.4, 15, 25.4), "#e8e8e8", 12),
        ("Cables y tornillería", (60, 0, Z_P1 + 4), (1, 1, 1), "#000000", 25),
    ]
    W = [("Rueda Ø65", (0, s*yw, Z_EJE), R_RUEDA, ANCHO_RUEDA, "#222", 36) for s in (1, -1)]
    return C, W

# ============================================================ MALLADO
def bbox_sdf(f, lim=(-60, 200, -70, 70, -30, 100), h=2.0):
    xs, ys, zs = np.arange(lim[0], lim[1], h), np.arange(lim[2], lim[3], h), np.arange(lim[4], lim[5], h)
    G = tuple(g.astype(np.float32) for g in np.meshgrid(xs, ys, zs, indexing="ij"))
    idx = np.argwhere(f(G) < h)
    lo, hi = idx.min(0), idx.max(0)
    return (xs[lo[0]]-4, xs[hi[0]]+4, ys[lo[1]]-4, ys[hi[1]]+4, zs[lo[2]]-4, zs[hi[2]]+4)

def mallar(f, h=H_GRID):
    x0, x1, y0, y1, z0, z1 = bbox_sdf(f)
    off = 0.1371 * h
    xs, ys, zs = np.arange(x0, x1, h)+off, np.arange(y0, y1, h)+off, np.arange(z0, z1, h)+off
    G = tuple(g.astype(np.float32) for g in np.meshgrid(xs, ys, zs, indexing="ij"))
    vol = f(G).astype(np.float32)
    vol[0], vol[-1], vol[:, 0], vol[:, -1], vol[:, :, 0], vol[:, :, -1] = (1,)*6
    v, fc, _, _ = marching_cubes(vol, 0.0, spacing=(h, h, h))
    v += np.array([x0, y0, z0]) + off
    if volumen(v, fc) < 0: fc = fc[:, ::-1]
    return v.astype(np.float32), fc.astype(np.int64)

def volumen(v, f):
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    return np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0

def centroide(v, f):
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    vi = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
    return ((a + b + c) / 4 * vi[:, None]).sum(0) / vi.sum()

def es_cerrada(f):
    e = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    _, cnt = np.unique(e, axis=0, return_counts=True)
    return bool(np.all(cnt == 2))

def guardar_stl(ruta, v, f):
    tri = v[f]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tri
    with open(ruta, "wb") as fh:
        fh.write(b"Chasis final seguidor de linea".ljust(80, b" "))
        fh.write(struct.pack("<I", len(f))); fh.write(rec.tobytes())

def para_imprimir(v, pieza):
    """Orienta cada pieza para la cama: boca abajo las que cuelgan, y apoyadas en z=0."""
    v = v.copy()
    if pieza in ("chasis", "carcasa"):
        v[:, 2] *= -1; v[:, 1] *= -1           # giro 180° sobre X
    v -= [ (v[:, 0].max()+v[:, 0].min())/2, (v[:, 1].max()+v[:, 1].min())/2, v[:, 2].min() ]
    return v

PIEZAS = {"chasis": chasis, "carcasa": carcasa}
PIEZAS_SUELTAS = {"calza_rueda": calza_rueda, "separadores": separadores}

def interferencias(fs, comps, h=0.7):
    res = []
    for nom, c, s, *_ in comps:
        if min(s) < 1.5: continue
        s2 = [max(si - 1.0, 0.5) for si in s]
        g = [np.arange(ci - si/2, ci + si/2 + 1e-6, h) for ci, si in zip(c, s2)]
        G = tuple(a.astype(np.float32) for a in np.meshgrid(*g, indexing="ij"))
        res.append((nom, round(float(max(0.0, -np.minimum.reduce([f(G) for f in fs]).min())), 2)))
    return res

def interferencia_ruedas(fs, rs, h=1.0):
    peor = 0.0
    for nom, c, R, W, *_ in rs:
        g = [np.arange(c[0]-R, c[0]+R, h), np.arange(c[1]-W/2+0.5, c[1]+W/2-0.5, h), np.arange(c[2]-R, c[2]+R, h)]
        G = tuple(a.astype(np.float32) for a in np.meshgrid(*g, indexing="ij"))
        m = np.hypot(G[0]-c[0], G[2]-c[2]) < R - 0.5
        peor = max(peor, float(max(0, -np.minimum.reduce([f(G) for f in fs])[m].min())))
    return peor

def barrido_rodachin(fs, h=0.8):
    """Cilindro que barre la horquilla al girar 360° (z 0..CASTER_H-3) no debe tocar el chasis."""
    g = [np.arange(CASTER_X-SWIVEL_R, CASTER_X+SWIVEL_R, h), np.arange(-SWIVEL_R, SWIVEL_R, h), np.arange(0.5, CASTER_H-3, h)]
    G = tuple(a.astype(np.float32) for a in np.meshgrid(*g, indexing="ij"))
    m = np.hypot(G[0]-CASTER_X, G[1]) < SWIVEL_R
    return float(max(0, -np.minimum.reduce([f(G) for f in fs])[m].min()))

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "stl_final"
    os.makedirs(out, exist_ok=True)
    info, masa_pla, mom = {}, 0.0, np.zeros(3)
    for nombre, f in {**PIEZAS, **PIEZAS_SUELTAS}.items():
        v, fc = mallar(f)
        vol = volumen(v, fc) / 1000.0
        g = vol * 1.24 * 0.55              # PLA, 3 paredes + 15 % relleno ≈ 55 % efectivo
        if nombre in PIEZAS:
            masa_pla += g; mom += g * centroide(v, fc)
        else:
            masa_pla += g; mom += g * np.array([CASTER_X, 0, 38.0])
        vp = para_imprimir(v, nombre)
        guardar_stl(os.path.join(out, f"final_{nombre}.stl"), vp, fc)
        info[nombre] = dict(cerrada=es_cerrada(fc), triangulos=int(len(fc)), vol_cm3=round(float(vol), 1),
                            masa_g=round(float(g), 1), cama_mm=[round(float(x), 1) for x in vp.max(0) - vp.min(0)])
        print(nombre, info[nombre])
    C, W = componentes()
    m_tot = masa_pla + sum(c[-1] for c in C) + sum(w[-1] for w in W)
    cg = (mom + sum(c[-1]*np.array(c[1]) for c in C) + sum(w[-1]*np.array(w[1]) for w in W)) / m_tot
    fs = list(PIEZAS.values())
    res = dict(piezas=info, masa_total_g=round(float(m_tot), 1), masa_pla_g=round(float(masa_pla), 1),
               cg_mm=[round(float(x), 1) for x in cg],
               interferencias=interferencias(fs, C), interf_ruedas=interferencia_ruedas(fs, W),
               barrido_rodachin=barrido_rodachin(fs),
               qre_x=QRE_X, caster_x=CASTER_X, caster_h=CASTER_H, calza_mm=round(Z_P0 - CASTER_H, 1))
    print(json.dumps({k: v for k, v in res.items() if k != "piezas"}, ensure_ascii=False))
    with open(os.path.join(out, "resumen_final.json"), "w", encoding="utf-8") as fh:
        json.dump(res, fh, ensure_ascii=False, indent=1)
