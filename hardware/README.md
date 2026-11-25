# 🖨️ Hardware mecánico

| Carpeta | Contenido |
|---|---|
| [`stl/`](stl) | Piezas listas para imprimir (ya orientadas en la cama) |
| [`cad/`](cad) | `chasis_final.py`: modelo paramétrico (SDF + marching cubes) · `analisis_rueda_loca.py`: comparación rueda loca adelante / atrás |

## Parámetros de impresión recomendados (Bambu Lab P1S)

| Parámetro | Valor |
|---|---|
| Material | PLA |
| Boquilla / capa | 0,4 mm / 0,2 mm |
| Paredes / relleno | 3 / 15 % |
| Soportes | **No** (base y carcasa se imprimen boca abajo) |

## Tolerancias

| Elemento | Nominal | Diseño |
|---|---|---|
| Tornillo M3 pasante | 3,0 | Ø3,4 |
| M3 autorroscante | 3,0 | Ø2,6 |
| Tuerca M3 (entre caras) | 5,5 | 5,8 × 2,6 prof. |
| Motor TT (espesor) | 18,8 | 19,3 |
| Portapilas 2×18650 | 41,0 | 41,6 |
| Transductor HC-SR04 | Ø16,0 | Ø16,6 |

## Regenerar el modelo

```bash
pip install numpy scikit-image
cd cad
python chasis_final.py ../stl          # CASTER_H=34 python chasis_final.py  (altura medida del rodachín)
python analisis_rueda_loca.py
```
