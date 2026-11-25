# app.py — Panel de control del robot seguidor de línea (Streamlit)
# Ejecutar en VS Code:  pip install -r requirements.txt   y luego   streamlit run app.py
# Desde el celular (misma red): http://<IP del PC>:8501
from pathlib import Path

import plotly.graph_objects as go
import streamlit as st
import streamlit.components.v1 as components

import cliente
import voz

st.set_page_config(page_title="Seguidor UMNG", page_icon="🤖", layout="wide")

COLORES_UI = {"negro": "#111111", "blanco": "#f4f4f4", "rojo": "#d62728", "amarillo": "#f2c200",
              "verde": "#2ca02c", "azul": "#1f5fd6", "desconocido": "#999999", "sin calibrar": "#cccccc"}

if "ultima_respuesta" not in st.session_state:
    st.session_state.ultima_respuesta = ""
if "voz_t" not in st.session_state:
    st.session_state.voz_t = 0

# componente propio (carpeta escucha_continua): micrófono siempre abierto, sin botón de enviar
escucha_continua = components.declare_component("escucha_continua",
                                                path=str(Path(__file__).parent / "escucha_continua"))


def enviar(cmd):
    r = cliente.comando(st.session_state.direccion, cmd)
    st.session_state.ultima_respuesta = ("✅ " if r.get("ok") else "⚠️ ") + (r.get("cmd") or r.get("msg", ""))
    return r


# ------------------------------------------------------------------ barra lateral
with st.sidebar:
    st.header("Conexión")
    st.text_input("Dirección del robot", value="seguidor.local", key="direccion",
                  help="IP que imprime el ESP32 al arrancar, o seguidor.local")
    st.caption("PC y robot deben estar en la misma red (hotspot del celular).")

    st.header("Calibración")
    c1, c2 = st.columns(2)
    if c1.button("1. Regleta", use_container_width=True, help="Robot sobre la línea: oscila 4 s"):
        enviar("cal_linea")
    if c2.button("2. Giro", use_container_width=True,
                 help="Robot centrado sobre un tramo RECTO, mirando a lo largo de la línea. Gira y mide su velocidad de giro"):
        enviar("cal_giro")
    c3, c4 = st.columns(2)
    if c3.button("Rumbo = 0°", use_container_width=True, help="Toma la dirección actual como 0°"):
        enviar("cero")
    if c4.button("Brújula", use_container_width=True, help="Solo si tienes una GY-271 conectada"):
        enviar("cal_brujula")
    col_sel = st.selectbox("Color bajo el sensor", ["negro", "blanco", "rojo", "amarillo", "verde", "azul"])
    if st.button("Guardar este color", use_container_width=True):
        enviar("cal_color:" + col_sel)

    st.header("Ajustes")
    vel = st.slider("Velocidad (%)", 20, 100, 45, 5)
    kp = st.number_input("Kp", 0.0, 200.0, 38.0, 1.0)
    kd = st.number_input("Kd", 0.0, 50.0, 3.2, 0.1)
    ajuste = st.number_input("Ajuste recto (%)", -20, 20, 0, 1,
                             help="Si en 'adelante' la rueda IZQUIERDA va más rápido, pon un valor NEGATIVO "
                                  "(-2, -4…). Si la DERECHA va más rápido, POSITIVO.")
    if st.button("Aplicar ajustes", use_container_width=True):
        enviar("vel:%d" % vel)
        enviar("pid:%g,%g" % (kp, kd))
        enviar("ajuste:%d" % ajuste)
    if st.button("Probar motores", use_container_width=True):
        enviar("prueba")

st.title("🤖 Seguidor de línea UMNG")

# ------------------------------------------------------------------ voz (primero: en el celular queda arriba)
st.subheader("🎙 Comando de voz")
oido = escucha_continua(key="escucha", default=None)
frase_continua = ""
if oido and oido.get("t") != st.session_state.voz_t:      # frase nueva (no repetir en cada recarga)
    st.session_state.voz_t = oido.get("t")
    frase_continua = oido.get("texto", "")
st.caption("Escucha continua: actívala una vez y habla; cada orden se envía sola. "
           "Solo en el PC con http://localhost:8501. En el celular usa la caja de abajo.")
with st.form("voz_form", clear_on_submit=True):
    escrito = st.text_input("Toca aquí, pulsa el micrófono del teclado del celular y di la orden",
                            placeholder="adelante · gira a la derecha · modo automático · para")
    enviado = st.form_submit_button("Enviar orden", use_container_width=True, type="primary")
texto_mic = None
try:                                    # botón de micrófono del navegador (funciona en el PC)
    from streamlit_mic_recorder import speech_to_text
    texto_mic = speech_to_text(language="es", start_prompt="🎙 Hablar (micrófono del navegador)",
                               stop_prompt="⏹ Enviar", just_once=True, use_container_width=True, key="voz")
except ImportError:
    pass
frase = frase_continua or texto_mic or (escrito if enviado else "")
if frase:
    cmd = voz.interpretar(frase)
    if cmd:
        enviar(cmd)
        st.success('"%s" → %s' % (frase, cmd))
    elif frase_continua:                # en escucha continua se oye de todo: no alarmar
        st.caption('(ignorado: "%s")' % frase)
    else:
        st.warning('No entendí: "%s"' % frase)
st.caption("Ejemplos: adelante · atrás · derecha · izquierda · para · modo automático · modo manual · "
           "gira 90 grados a la derecha · rumbo 180 · reinicia el rumbo · velocidad 60")

# ------------------------------------------------------------------ botones
st.subheader("Modo")
a, b, c = st.columns(3)
if a.button("▶ Autónomo", use_container_width=True, type="primary"):
    enviar("auto")
if b.button("🕹 Manual", use_container_width=True):
    enviar("manual")
if c.button("⏹ PARAR", use_container_width=True):
    enviar("parar")

st.subheader("Control manual")
f1, f2, f3, f4, f5 = st.columns(5)
if f1.button("⬆ Adelante", use_container_width=True):
    enviar("adelante")
if f2.button("⬅ Izquierda", use_container_width=True):
    enviar("izquierda")
if f3.button("⏹ Parar", use_container_width=True, key="parar2"):
    enviar("parar")
if f4.button("Derecha ➡", use_container_width=True):
    enviar("derecha")
if f5.button("⬇ Atrás", use_container_width=True):
    enviar("atras")
r1, r2 = st.columns([3, 1])
rumbo_obj = r1.slider("Rumbo deseado (°)", 0, 359, 0, 5)
if r2.button("Orientar", use_container_width=True):
    enviar("rumbo:%d" % rumbo_obj)
st.write(st.session_state.ultima_respuesta)


# ------------------------------------------------------------------ telemetría en tiempo real
def brujula_fig(actual, deseado):
    fig = go.Figure()
    fig.add_trace(go.Scatterpolar(r=[0, 1], theta=[deseado, deseado], mode="lines",
                                  line=dict(color="#1f77b4", width=4, dash="dash"), name="Deseado %.0f°" % deseado))
    fig.add_trace(go.Scatterpolar(r=[0, 1], theta=[actual, actual], mode="lines+markers",
                                  line=dict(color="#d62728", width=6), marker=dict(size=[0, 14]),
                                  name="Actual %.0f°" % actual))
    fig.update_layout(polar=dict(angularaxis=dict(rotation=90, direction="clockwise",
                                                  tickvals=[0, 45, 90, 135, 180, 225, 270, 315],
                                                  ticktext=["N", "45", "E", "135", "S", "225", "O", "315"]),
                                 radialaxis=dict(visible=False, range=[0, 1])),
                      height=330, margin=dict(l=30, r=30, t=30, b=10),
                      legend=dict(orientation="h", y=-0.05))
    return fig


@st.fragment(run_every="500ms")
def panel():
    e = cliente.estado(st.session_state.direccion)
    if not e:
        st.error("Sin conexión con el robot en %s. Revisa la red y la dirección." % st.session_state.direccion)
        return
    k = st.columns(6)
    k[0].metric("Modo", e["modo"])
    k[1].metric("Estado", e["estado"] if not e["calibrando"] else "CAL " + e["calibrando"])
    k[2].metric("Rumbo", "%.0f°" % e["rumbo"])
    k[3].metric("Color", e["color"])
    d = e.get("distancia")
    k[4].metric("Distancia", "—" if d is None else "%.0f cm" % d)
    k[5].metric("Esquivados", e.get("obstaculos", 0))

    g1, g2, g3 = st.columns([1.1, 1.2, 1])
    with g1:
        st.plotly_chart(brujula_fig(e["rumbo"], e["rumbo_deseado"]), use_container_width=True, key="polar")
        st.caption("Rumbo %s · giro %.0f °/s" % (e.get("rumbo_fuente", e["brujula"]), e.get("tasa_giro", 0)))
    with g2:
        bar = go.Figure(go.Bar(x=["S%d" % (i + 1) for i in range(8)], y=e["linea"],
                               marker_color=["#222" if v > 500 else "#bbb" for v in e["linea"]]))
        bar.add_hline(y=300, line_dash="dot", line_color="#999")
        bar.update_layout(height=250, margin=dict(l=10, r=10, t=30, b=10), yaxis=dict(range=[0, 1000]),
                          title="Regleta (posición %d · error %.2f)" % (e["posicion"], e["error_linea"]))
        st.plotly_chart(bar, use_container_width=True, key="regleta")
        st.caption("Motores: izq %d %% · der %d %%" % tuple(e["vel"]))
    with g3:
        col = e["color"]
        r, g, b = e["rgb"]
        st.markdown(
            '<div style="display:flex;gap:10px;align-items:center">'
            '<div style="width:56px;height:56px;border-radius:8px;border:1px solid #888;background:%s"></div>'
            '<div><b style="font-size:1.4em">%s</b><br><span style="opacity:.7">RGB aprox. %d, %d, %d</span></div></div>'
            % (COLORES_UI.get(col, "rgb(%d,%d,%d)" % (r, g, b)), col.upper(), r, g, b), unsafe_allow_html=True)
        st.write("**Colores recorridos**")
        st.write(" → ".join(c for _, c in e["colores"]) or "—")
        cal = e["calibrado"]
        st.caption("Calibrado: regleta %s · giro %s · colores %s" % (
            "✔" if cal["linea"] else "✘", "✔" if cal.get("giro") or cal["brujula"] else "✘",
            ", ".join(cal["colores"]) or "ninguno"))
        with st.expander("Eventos"):
            for t, txt in reversed(e["eventos"]):
                st.write("`%6.1f s` %s" % (t, txt))


panel()
