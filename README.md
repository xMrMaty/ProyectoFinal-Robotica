# Proyecto Final Robotica: Navegación Autónoma con Planificación de Rutas (Línea A)
**Integrantes:** Matías Ruiz Flores, Joaquín Castro Delgado, Álvaro Del Pino Cerda.

## 1. Objetivo del Proyecto
[Describe cómo integrarán el control cinemático, los sensores de distancia y la planificación $A^*$ para llegar a la meta]

## 2. Descripción del Hardware Simulado
* [cite_start]**Robot:** e-puck [cite: 87]
* [cite_start]**Sensores:** 8 sensores de proximidad infrarrojos, encoders de rueda[cite: 89].
* **Actuadores:** 2 motores de corriente continua (diferenciales).

## 3. Escenarios de Prueba
1.  [cite_start]**Escenario Simple:** Pocos obstáculos, ruta directa[cite: 96].
2.  [cite_start]**Escenario Complejo:** Pasillos estrechos, zonas de bloqueo, múltiples obstáculos[cite: 97].

## 4. Algoritmo Implementado ($A^*$)
[Explica cómo discretizaron el mapa en una grilla y cómo $A^*$ calcula la ruta más corta. Incluye un diagrama de flujo]

## 5. Relación con Laboratorios 1 y 2
[Explica que reutilizaron la cinemática diferencial (Lab 1) y la evitación de obstáculos / filtrado (Lab 2)]

## 6. Resultados y Métricas
* Tiempo total hasta la meta: X segundos.
* Número de colisiones: X.
* [Añadir gráficos de la trayectoria real vs planificada]

## 7. Instrucciones de Ejecución
1. Clonar el repositorio.
2. Abrir el archivo `mundo_complejo.wbt` en Webots.
3. Ejecutar la simulación.
