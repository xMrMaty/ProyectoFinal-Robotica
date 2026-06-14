# Proyecto Final: Navegación Autónoma con Planificación de Rutas (A*)

**Integrantes:**
- Matías Ruiz Flores
- Joaquín Castro Delgado
- Álvaro Del Pino Cerda

**Línea seleccionada:** Línea A — Planificación de rutas con algoritmo A*

---

## 1. Objetivo del Proyecto

Implementar un sistema de navegación autónoma para un robot móvil diferencial (e-puck) en un entorno con obstáculos, utilizando planificación de rutas basada en el algoritmo A* sobre una grilla de ocupación, control cinemático diferencial, odometría por encoders y evasión reactiva de obstáculos mediante sensores infrarrojos y visión artificial.

---

## 2. Descripción del Robot, Sensores y Actuadores

- **Robot:** e-puck (robot móvil diferencial simulado en Webots)
- **Sensores de proximidad:** 8 sensores infrarrojos (PS0–PS7) distribuidos alrededor del chasis, utilizados para detección de obstáculos y evasión reactiva
- **Encoders:** Sensores de posición en ambas ruedas (`left wheel sensor` / `right wheel sensor`) para estimación de odometría
- **Cámara frontal:** Cámara RGB integrada, utilizada para detección visual de la marca de meta (bloque con color emisivo rojo)
- **Actuadores:** Dos motores de corriente continua independientes (`left wheel motor` / `right wheel motor`) con control de velocidad

---

## 3. Escenarios de Prueba

Se diseñaron dos escenarios distintos en Webots sobre una arena de **2x2 metros** (`RectangleArena`):

### Escenario Simple (`escenario_simple.wbt`)
Tres obstáculos sólidos (nodos `Solid` con `boundingObject`) ubicados en el área central, dejando pasillos amplios hacia la meta. Permite verificar el funcionamiento base del algoritmo A* y la cinemática diferencial sin condiciones restrictivas.

### Escenario Complejo (`escenario_complejo.wbt`)
Laberinto de pasillos estrechos formado por paredes sólidas en disposición de "L", que obliga al robot a planificar una ruta no directa desde la esquina inferior izquierda hasta la esquina superior derecha. La meta está marcada con un bloque de color rojo emisivo para su detección por cámara.

- **Posición inicial del robot:** `(-0.82, -0.82)`, orientación `0 1 0 0`
- **Meta:** esquina superior derecha, marcada visualmente con bloque rojo (`emissiveColor: 1 0 0`)

---

## 4. Algoritmo Implementado

### Grilla de Ocupación
El entorno de 2x2 metros se discretizó en una matriz 10x10 (celdas de 20x20 cm). Cada celda toma valor `0` (libre) o `1` (obstáculo). Los obstáculos físicos del laberinto se representan como bloques de `1` en las filas y columnas correspondientes.

### A* (A-Star)
El algoritmo A* calcula la ruta de menor costo desde la celda de inicio hasta la celda meta, usando distancia euclidiana como heurística. La ruta resultante (lista de celdas) se convierte a coordenadas en metros mediante la función `celda_a_metros()`, generando una secuencia de waypoints que el robot debe seguir en orden.

### Control Cinemático (Seguimiento de Waypoints)
Para cada waypoint, se calcula el ángulo hacia el objetivo (`atan2`) y el error angular respecto a la orientación actual del robot. Un controlador proporcional ajusta las velocidades de ambas ruedas:
v_l = (v - w * L/2) / r
v_r = (v + w * L/2) / r
donde `v` es la velocidad lineal base, `w` el error angular corregido, `L` la distancia entre ruedas y `r` el radio de rueda.

### Máquina de Estados
El robot opera con dos estados:
- **NAVEGANDO:** sigue la ruta calculada por A*
- **ESQUIVANDO:** al detectar un obstáculo inminente por sensores IR, interrumpe el seguimiento de ruta y ejecuta una maniobra evasiva hacia el lado con menor lectura de sensores

---

## 5. Diagrama de Flujo
INICIO
│
├─► Calcular ruta con A* (grilla → waypoints en metros)
└─► Bucle de simulación:
├─► Leer encoders → Actualizar odometría (x, y, φ)
├─► Leer sensores IR → ¿Peligro inminente?
│     ├─ SÍ → Estado: ESQUIVANDO → Girar al lado más despejado
│     └─ NO → Estado: NAVEGANDO
├─► Leer cámara → ¿Detecta bloque rojo (>15 píxeles)?
│     └─ SÍ → Detener motores → FIN (Meta visual alcanzada)
├─► ¿Distancia a meta < 0.15m o waypoints agotados?
│     └─ SÍ → Detener motores → FIN (Meta odométrica alcanzada)
└─► Calcular error angular → Ajustar v_l, v_r → Aplicar a motores

---

## 6. Relación con Laboratorios Anteriores

**Laboratorio 1 — Cinemática diferencial:**
Se reutilizó el modelo de control de velocidades para robot diferencial, aplicando las ecuaciones de conversión entre velocidad lineal/angular y velocidades de rueda individual (`v_l`, `v_r`).

**Laboratorio 2 — Odometría y sensores:**
Se integró la estimación de posición basada en encoders de rueda (odometría incremental) mediante las ecuaciones:
Δsl = r · Δθl
Δsr = r · Δθr
Δs  = (Δsr + Δsl) / 2
Δφ  = (Δsr - Δsl) / L
x  += Δs · cos(φ + Δφ/2)
y  += Δs · sin(φ + Δφ/2)
φ  += Δφ

Además, se reutilizó la lógica de lectura y procesamiento de sensores infrarrojos para la detección y evasión reactiva de obstáculos.

---

## 7. Resultados y Métricas de Desempeño

| Métrica | Escenario Simple | Escenario Complejo |
|---|---|---|
| Tiempo hasta la meta | ~30 segundos | ~55 segundos |
| Colisiones físicas | 0 | 0 |
| Activaciones de evasión | 1–2 | 4–6 |
| Error odométrico final | ~0.10 m | ~0.18 m |
| Detección de meta | Visual (cámara) | Visual (cámara) |

### Comparación Ruta Planificada vs Trayectoria Ejecutada

El gráfico a continuación fue generado a partir del archivo `datos_ruta.csv` exportado automáticamente al finalizar la simulación:

- **Línea azul punteada:** Ruta calculada por el algoritmo A* (waypoints ideales)
- **Línea roja continua:** Trayectoria real registrada mediante odometría

<img width="711" height="702" alt="image" src="https://github.com/user-attachments/assets/ee8e7902-a17b-4fc2-9570-80c5822e468f" />


Las desviaciones observadas en la zona inicial se explican por la activación del estado de evasión reactiva al detectar proximidad con las paredes perimetrales, generando un error odométrico momentáneo que el sistema logra compensar retomando la ruta planificada.

---

## 8. Instrucciones de Ejecución

1. Instalar [Webots](https://cyberbotics.com/) (versión R2023b o superior)
2. Clonar este repositorio
3. Abrir `worlds/escenario_complejo.wbt` en Webots
4. Verificar que el campo `controller` del nodo E-puck esté asignado a `ControladorFinal`
5. Presionar el botón **Reset** (`|◀◀`) para ubicar el robot en la posición inicial
6. Presionar **Play** (`▶`) para iniciar la simulación
7. Al finalizar, se generará automáticamente el archivo `datos_ruta.csv` en la carpeta del controlador
8. Para generar el gráfico comparativo, ejecutar desde terminal:
```bash
   pip install matplotlib
   python graficar.py
```

---

## 9. Conclusiones, Limitaciones y Posibles Mejoras

### Conclusiones
- El algoritmo A* planificó correctamente rutas óptimas evitando zonas de obstáculos, generando trayectorias en forma de "L" coherentes con la estructura del laberinto
- La integración de visión artificial como sistema de detección de meta resultó más robusta que depender exclusivamente de odometría, la cual acumula error progresivamente por el patinaje de ruedas en curvas y maniobras de evasión
- La máquina de estados permitió combinar navegación global (A*) con evasión local (sensores IR) sin que ambos sistemas se bloquearan mutuamente

### Limitaciones
- La odometría pura pierde precisión en trayectorias largas o con muchas correcciones, sin un sistema de localización absoluta de respaldo
- La grilla de ocupación discreta (10x10) limita la granularidad de la ruta planificada, lo que puede generar colisiones en pasillos muy estrechos si el mapa no está bien calibrado
- La detección visual por color es sensible a cambios de iluminación en el entorno simulado

### Posibles Mejoras
- Implementar un **filtro de Kalman** para fusionar datos de odometría con sensores adicionales (GPS simulado), reduciendo el error acumulado
- Aumentar la resolución de la grilla de ocupación a 20x20 o superior para mayor precisión en la planificación
- Añadir **re-planificación dinámica** cuando los sensores detecten obstáculos no representados en el mapa inicial

---

*Documentación desarrollada para el curso de Robótica y Sistemas Autónomos — 2026*
