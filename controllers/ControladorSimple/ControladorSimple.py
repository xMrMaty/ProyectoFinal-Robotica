from controller import Robot
import math

# --- CONSTANTES ---
TIME_STEP = 64
MAX_SPEED = 6.28
WHEEL_RADIUS = 0.0205
WHEEL_BASE = 0.052

# --- ALGORITMO A* ---
class Nodo:
    def __init__(self, padre=None, posicion=None):
        self.padre = padre
        self.posicion = posicion
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, otro):
        return self.posicion == otro.posicion

def a_star(grilla, inicio, meta):
    lista_abierta = [Nodo(None, inicio)]
    lista_cerrada = []
    movimientos = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    while lista_abierta:
        nodo_actual = min(lista_abierta, key=lambda o: o.f)
        lista_abierta.remove(nodo_actual)
        lista_cerrada.append(nodo_actual)

        if nodo_actual.posicion == meta:
            camino = []
            while nodo_actual:
                camino.append(nodo_actual.posicion)
                nodo_actual = nodo_actual.padre
            return camino[::-1]

        for mov in movimientos:
            pos = (nodo_actual.posicion[0] + mov[0], nodo_actual.posicion[1] + mov[1])

            if pos[0] < 0 or pos[0] >= len(grilla) or pos[1] < 0 or pos[1] >= len(grilla[0]):
                continue
            if grilla[pos[0]][pos[1]] != 0:
                continue

            nuevo_nodo = Nodo(nodo_actual, pos)
            if nuevo_nodo in lista_cerrada:
                continue

            nuevo_nodo.g = nodo_actual.g + math.sqrt(mov[0]**2 + mov[1]**2)
            nuevo_nodo.h = math.sqrt((pos[0] - meta[0])**2 + (pos[1] - meta[1])**2)
            nuevo_nodo.f = nuevo_nodo.g + nuevo_nodo.h

            if any(abierto == nuevo_nodo and nuevo_nodo.g > abierto.g for abierto in lista_abierta):
                continue
            lista_abierta.append(nuevo_nodo)
    return None

def celda_a_metros(fila, columna, tamano_celda=0.2, offset=-1.0):
    x = offset + (columna * tamano_celda) + (tamano_celda / 2.0)
    y = offset + (fila * tamano_celda) + (tamano_celda / 2.0)
    return (x, y)

# --- FUNCIÓN PRINCIPAL ---
def main():
    robot = Robot()

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')
    left_encoder.enable(TIME_STEP)
    right_encoder.enable(TIME_STEP)

    ps = [robot.getDevice(f'ps{i}') for i in range(8)]
    for sensor in ps:
        sensor.enable(TIME_STEP)

    camara = robot.getDevice('camera')
    camara.enable(TIME_STEP)
    width = camara.getWidth()
    height = camara.getHeight()

    # Posición inicial: esquina inferior izquierda, mirando a la derecha
    x, y, phi = -0.82, -0.82, 0.0
    last_ps_l, last_ps_r = 0.0, 0.0

    # Grilla 10x10 para escenario simple
    # 3 obstáculos verdes en posiciones centrales, resto libre
    mapa = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # Obstáculo izquierda-centro
        [0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 0, 0, 0],  # Obstáculo centro
        [0, 0, 0, 0, 0, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ]

    inicio_grid = (9, 0)  # Esquina inferior izquierda
    meta_grid = (0, 9)    # Esquina superior derecha

    print("Calculando ruta con A*...")
    ruta_celdas = a_star(mapa, inicio_grid, meta_grid)

    if ruta_celdas is None:
        print("ERROR: Ruta bloqueada. Revisa la matriz.")
        return

    ruta_metros = [celda_a_metros(9 - r[0], r[1]) for r in ruta_celdas]
    print(f"Ruta encontrada con {len(ruta_metros)} waypoints. Navegando...")

    waypoint_actual = 0
    estado = "NAVEGANDO"

    while robot.step(TIME_STEP) != -1:
        # 1. ODOMETRÍA
        ps_l = left_encoder.getValue()
        ps_r = right_encoder.getValue()
        delta_sl = WHEEL_RADIUS * (ps_l - last_ps_l)
        delta_sr = WHEEL_RADIUS * (ps_r - last_ps_r)
        last_ps_l, last_ps_r = ps_l, ps_r

        delta_s = (delta_sr + delta_sl) / 2.0
        delta_phi = (delta_sr - delta_sl) / WHEEL_BASE

        x += delta_s * math.cos(phi + delta_phi / 2.0)
        y += delta_s * math.sin(phi + delta_phi / 2.0)
        phi += delta_phi

        # 2. SENSORES IR
        ps_values = [s.getValue() for s in ps]
        peligro = ps_values[0] > 150 or ps_values[7] > 150 or ps_values[1] > 200 or ps_values[6] > 200
        obstaculo_izq = ps_values[7] + ps_values[6] + ps_values[5]
        obstaculo_der = ps_values[0] + ps_values[1] + ps_values[2]

        # 3. VISIÓN ARTIFICIAL - Detección del bloque rojo
        imagen = camara.getImageArray()
        if imagen:
            pixeles_rojos = 0
            for pixel_x in range(width):
                for pixel_y in range(height):
                    r = imagen[pixel_x][pixel_y][0]
                    g = imagen[pixel_x][pixel_y][1]
                    b = imagen[pixel_x][pixel_y][2]
                    if r > 130 and g < 80 and b < 80:
                        pixeles_rojos += 1

            if pixeles_rojos > 15:
                print(f"Meta visual detectada ({pixeles_rojos} px). Mision cumplida.")
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                break

        # 4. MÁQUINA DE ESTADOS
        v_l, v_r = 0.0, 0.0

        # Respaldo odométrico
        meta_x, meta_y = ruta_metros[-1]
        if math.sqrt((meta_x - x)**2 + (meta_y - y)**2) < 0.15 or waypoint_actual >= len(ruta_metros):
            print("Meta odometrica alcanzada.")
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            break

        if peligro:
            estado = "ESQUIVANDO"
            if obstaculo_izq > obstaculo_der:
                v_l = MAX_SPEED * 0.5
                v_r = -MAX_SPEED * 0.5
            else:
                v_l = -MAX_SPEED * 0.5
                v_r = MAX_SPEED * 0.5
        else:
            if estado == "ESQUIVANDO":
                estado = "NAVEGANDO"

            target_x, target_y = ruta_metros[waypoint_actual]
            error_x = target_x - x
            error_y = target_y - y

            if math.sqrt(error_x**2 + error_y**2) < 0.20:
                waypoint_actual += 1
                continue

            angulo_meta = math.atan2(error_y, error_x)
            error_angular = math.atan2(math.sin(angulo_meta - phi), math.cos(angulo_meta - phi))

            if abs(error_angular) < 0.15:
                error_angular = 0.0

            v_l = (0.15 - (0.3 * error_angular * WHEEL_BASE / 2.0)) / WHEEL_RADIUS
            v_r = (0.15 + (0.3 * error_angular * WHEEL_BASE / 2.0)) / WHEEL_RADIUS

        left_motor.setVelocity(max(min(v_l, MAX_SPEED), -MAX_SPEED))
        right_motor.setVelocity(max(min(v_r, MAX_SPEED), -MAX_SPEED))

if __name__ == "__main__":
    main()