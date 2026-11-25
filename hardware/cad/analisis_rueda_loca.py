"""
Análisis: rueda loca ADELANTE (config. B, chasis final) vs ATRÁS (config. A)
Mismo robot, mismos componentes, misma distancia eje->regleta (d = 106 mm).
Modelo cuasi-estático de cuerpo rígido con 3 apoyos (2 ruedas + rueda loca).
"""
import json, sys
import numpy as np
sys.path.insert(0, "/home/claude")
import chasis_final as CF

g, MU = 9.81, 0.7                      # fricción caucho TT sobre pista de vinilo/cartulina
TAU_STALL = 0.8 * 0.0981 * (5.6 / 6)   # N·m por motor (0.8 kgf·cm @6 V, ~5.6 V tras el L298N)
F_MOT = 2 * TAU_STALL / CF.R_RUEDA * 1000  # N (fuerza máxima en el piso, ambos motores)
Y_RUEDA = CF.Y_INT + CF.MOTOR_W + 3 + CF.ANCHO_RUEDA / 2
TRAIL = 8.0                            # avance de la horquilla del rodachín

def masas_B():
    res = json.load(open(sys.argv[1]))
    C, W = CF.componentes()
    return res["masa_total_g"], np.array(res["cg_mm"])

def masas_A():
    """Config. A: rueda loca ATRÁS. Para que el CG quede entre la rueda loca y el eje
    hay que llevar motores, batería y L298N hacia atrás (es la única forma estable)."""
    L = [  # masa_g, x, z
        (30.6*2, -23.5, 32.5),          # motores con cuerpo hacia atrás
        (36*2,     0.0, 32.5),          # ruedas
        (107,    -24.0, 31.6),          # portapilas entre motores
        (30,     -45.0, 64.7),          # L298N atrás, sobre la batería
        (30,     -10.0, 63.7),          # ESP32 sobre el eje
        (10,     120.0, 56.5),          # HC-SR05 al frente
        (9,       92.0, 10.0),          # QRE + TCS
        (20,     -95.0, 17.0),          # rodachín atrás
        (25,       0.0, 49.0),          # cables
        (136,     -5.0, 52.0),          # chasis+carcasa (más largo atrás para alojar la rueda loca)
    ]
    m = sum(a for a, _, _ in L)
    return m, np.array([sum(a*x for a, x, _ in L)/m, 0, sum(a*z for a, _, z in L)/m])

def metricas(nombre, m_g, cg, x_c, x_s=CF.QRE_X):
    m = m_g / 1000
    xg, h = cg[0], cg[2]
    xc_c = x_c - TRAIL                              # contacto del rodachín (va arrastrado)
    W = m * g
    Nw = W * (xc_c - xg) / xc_c                     # carga estática en ruedas motrices
    a_trac = MU * g * (xc_c - xg) / (xc_c - MU * h)
    a_mot = F_MOT / m
    a_disp = min(a_trac, a_mot)
    # vuelco longitudinal (despega la rueda loca)
    if xc_c > 0:   # B: al acelerar se levanta la nariz (caballito)
        a_cab = g * xg / h; modo = "caballito al acelerar (sensores suben)"
    else:          # A: al frenar se clava la nariz (sensores bajan)
        a_cab = g * abs(xg) / h; modo = "clavada al frenar (sensores bajan)"
    a_fren = MU * g * (xc_c - xg) / (xc_c + MU * h)
    # sensibilidad de la altura de la regleta (mm por mm)
    k_rueda = x_s / xc_c                            # bache/tolerancia bajo la rueda loca
    k_motriz = (xc_c - x_s) / xc_c                  # bache bajo ambas ruedas motrices
    # vuelco lateral: distancia del CG a los bordes del triángulo de apoyo
    P1, P2, Pc = np.array([0, Y_RUEDA]), np.array([0, -Y_RUEDA]), np.array([xc_c, 0])
    def dist(p, a, b):
        u, w = b - a, p - a
        return abs(u[0]*w[1] - u[1]*w[0]) / np.linalg.norm(u)
    dmin = min(dist(np.array([xg, 0]), P1, Pc), dist(np.array([xg, 0]), P2, Pc))
    a_lat_vuelco = g * dmin / h
    a_lat_agarre = MU * g * Nw / W          # la rueda loca giratoria no aporta fuerza lateral
    ks = [k for k in (k_rueda, k_motriz) if k > 0]
    bache_pierde = round((9.5 - CF.QRE_H) / max(ks), 1)
    # rigidez del soporte de sensores (misma sección de placa; relación de flechas)
    if xc_c > 0:   # viga apoyada entre eje y rueda loca, carga en x_s
        flecha_rel = (x_s**2 * (xc_c - x_s)**2) / (3 * xc_c)
    else:          # voladizo desde el eje
        flecha_rel = x_s**3 / 3
    return dict(config=nombre, masa_g=round(m_g), cg_x=round(xg, 1), cg_z=round(h, 1), x_rueda_loca=x_c,
                carga_ruedas_pct=round(100 * Nw / W, 1), a_traccion=round(a_trac, 2), a_motores=round(a_mot, 2),
                a_disponible=round(a_disp, 2), a_vuelco_long=round(a_cab, 2), modo_vuelco=modo,
                margen_vuelco=round(a_cab / (a_disp if xc_c > 0 else min(a_fren, a_mot)), 2), a_frenado=round(a_fren, 2),
                k_rueda_loca=round(k_rueda, 2), k_motrices=round(k_motriz, 2),
                bache_que_toca_piso_mm=(round(CF.QRE_H / abs(k_rueda), 1) if k_rueda < 0 else None),
                bache_pierde_lectura_mm=bache_pierde,
                a_lat_vuelco=round(a_lat_vuelco, 2), a_lat_agarre=round(a_lat_agarre, 2),
                limite_curva="subviraje (sale de nariz, estable)" if xg > 0 else "sobreviraje (cola sale, trompo)",
                flecha_rel=flecha_rel)

mB, cgB = masas_B()
mA, cgA = masas_A()
A = metricas("A · rueda loca atrás", mA, cgA, -95.0)
B = metricas("B · rueda loca adelante", mB, cgB, CF.CASTER_X)
ratio = A["flecha_rel"] / B["flecha_rel"]
for r in (A, B): r.pop("flecha_rel")

# curvas: con la regleta a d del eje, el eje recorre r = sqrt(R² - d²)
d = CF.QRE_X
curvas = [dict(R=R, r_eje=(round(float(np.sqrt(R**2 - d**2)), 1) if R > d else None)) for R in (100, 120, 150, 200, 300)]

# compatibilidad de las 5 propuestas anteriores con el rodachín (altura CASTER_H)
prev = {"P1": (20.1, 95, -97), "P2": (42.2, 150, -88), "P3": (20.1, 95, -97), "P4": (15.0, 95, -98), "P5": (20.1, 120, -97)}
compat = {}
for k, (cara, xs, xc) in prev.items():
    exceso = CF.CASTER_H - cara                    # >0: el rodachín es más alto que la cara de montaje
    if exceso <= 0:
        compat[k] = dict(cara=cara, calza=round(-exceso, 1), h_qre=3.0, ok=True)
    else:                                          # el chasis se inclina con la nariz hacia abajo
        h = 3.0 - exceso * xs / abs(xc)
        compat[k] = dict(cara=cara, calza=0, h_qre=round(h, 1), ok=h > 1)

out = dict(A=A, B=B, rigidez_B_sobre_A=round(ratio, 1), F_motores_N=round(F_MOT, 2), mu=MU,
           curvas=curvas, compat_prev=compat, d_mm=round(d, 1))
print(json.dumps(out, ensure_ascii=False, indent=1))
json.dump(out, open(sys.argv[2], "w"), ensure_ascii=False, indent=1)
