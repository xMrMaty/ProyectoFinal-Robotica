Proyecto Final: Planificación de Rutas Autónomas en Entornos Complejos
Integrantes:

Matias Ruiz Flores

Joaquín Castro Delgado

Álvaro Del Pino Cerda

Línea seleccionada: Planificación de rutas (A*).

1. Objetivo del Proyecto
Implementar un sistema de navegación autónoma para un robot móvil diferencial en un entorno con obstáculos, utilizando planificación de rutas basada en el algoritmo A* y evasión reactiva de obstáculos mediante sensores infrarrojos.

2. Descripción del Robot
Modelo: e-puck (robot móvil diferencial).

Sensores: 8 sensores de proximidad infrarrojos (PS0-PS7) para detección de obstáculos y una cámara frontal para reconocimiento de marcas visuales.

Actuadores: Dos motores de CC independientes con encoders de alta resolución para odometría.

3. Escenarios de Prueba
El entorno consta de una arena de 2x2 metros diseñada en Webots, caracterizada por una configuración de pasillos estrechos y obstáculos sólidos. Se definió una posición inicial fija en la esquina inferior izquierda y una meta marcada mediante un bloque con emisividad roja en la esquina superior derecha.

4. Algoritmo Implementado
Se utilizó A* sobre una grilla de ocupación discreta (10x10), calculando la trayectoria de menor costo.

Manejo de estados: Se implementó una máquina de estados que prioriza el seguimiento de ruta, pero que delega el control a una rutina de evasión reactiva ante colisiones inminentes, superando las limitaciones de la odometría pura frente al derrape físico.

Diagrama de Flujo
5. Relación con Laboratorios previos
Laboratorio 1: Integración de la cinemática diferencial y el modelo de movimiento del robot.

Laboratorio 2: Aplicación de odometría basada en encoders y control proporcional para el seguimiento de waypoints.

6. Resultados y Métricas
El robot logró navegar de forma autónoma con un error de posicionamiento compensado por visión artificial.

Métrica principal: Desviación de trayectoria.

<img width="711" height="702" alt="image" src="https://github.com/user-attachments/assets/e9950947-c128-4764-8394-7dfc18460e8f" />


Resultado: La trayectoria ejecutada (línea roja) sigue el patrón planificado (línea azul) con correcciones reactivas en los nodos de mayor densidad de obstáculos.

7. Instrucciones de Ejecución
Instalar Webots (versión R2023b o superior).

Clonar el repositorio.

Abrir el archivo EscenarioComplejo.wbt en Webots.

Asegurar que el controlador ControladorFinal.py esté seleccionado en el robot.

Presionar el botón Reset (|◀◀) y luego Play (▶).

8. Conclusiones y Limitaciones
Conclusiones: La integración de visión artificial como "freno de emergencia" resultó más robusta que la odometría pura, la cual tiende a acumular errores debido al patinaje de las ruedas en curvas.

Limitaciones: El algoritmo A* en una grilla discreta limitada restringe la precisión del movimiento en pasillos estrechos.

Mejoras posibles: Implementar un filtro de Kalman para la fusión de datos entre GPS (simulado) y odometría, permitiendo mayor precisión sin depender exclusivamente de landmarks visuales.
