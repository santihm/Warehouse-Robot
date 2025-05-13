#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import WarehouseMap2, RobotStatus

# Variables globales
current_robot_position = [1.0, 1.0]  # Posición inicial del robot en una posición válida

def robot_status_callback(status_msg):
    global current_robot_position
    current_robot_position = [status_msg.x, status_msg.y]
    publish_map()

def publish_map():
    global current_robot_position

    # Crear una estructura de mapa de 20x20
    map_data = WarehouseMap2()

    # Establecer las estaciones de carga y descarga en el norte y sur
    map_data.stations = [
        9.0, 0.0,  # Estación 1 (norte)
        10.0, 0.0, # Estación 2 (norte)
        9.0, 19.0, # Estación 3 (sur)
        10.0, 19.0 # Estación 4 (sur)
    ]

    # Distribuir las estanterías en diferentes partes del mapa creando pasillos
    map_data.shelves = []
    for x in range(2, 18, 2):
        for y in range(2, 18, 2):
            if not (9 <= x <= 10 and (y == 0 or y == 19)):  # Evitar las estaciones
                map_data.shelves.extend([float(x), float(y)])

    # Muros: Todos los bordes del mapa (20x20), excepto las 4 celdas en el norte y sur
    map_data.walls = []
    
    # Muros en las filas superiores e inferiores (excepto donde están las estaciones)
    for x in range(20):
        if x < 9 or x > 10:  # Aseguramos que las estaciones no sean muros
            map_data.walls.extend([float(x), 0.0])  # Pared norte
            map_data.walls.extend([float(x), 19.0]) # Pared sur

    # Muros en las columnas izquierda y derecha (en todo el mapa)
    for y in range(20):
        map_data.walls.extend([0.0, float(y)])  # Pared izquierda
        map_data.walls.extend([19.0, float(y)]) # Pared derecha

    # Posición actual del robot
    map_data.robot_position = current_robot_position

    # Crear una representación visual del mapa
    map_visual = [["." for _ in range(20)] for _ in range(20)]  # Mapa vacío con puntos (espacios vacíos)

    # Marcar los muros
    for i in range(0, len(map_data.walls), 2):
        x, y = int(map_data.walls[i]), int(map_data.walls[i+1])
        map_visual[y][x] = "#"  # Colocamos un muro

    # Marcar las estaciones de carga y descarga
    for i in range(0, len(map_data.stations), 2):
        x, y = int(map_data.stations[i]), int(map_data.stations[i+1])
        map_visual[y][x] = "S"  # Colocamos una estación

    # Marcar las estanterías
    for i in range(0, len(map_data.shelves), 2):
        x, y = int(map_data.shelves[i]), int(map_data.shelves[i+1])
        map_visual[y][x] = "E"  # Colocamos una estantería

    # Marcar la posición actual del robot
    robot_x, robot_y = int(current_robot_position[0]), int(current_robot_position[1])
    map_visual[robot_y][robot_x] = "R"  # Colocamos el robot

    # Mostrar la representación visual del mapa en la consola
    #rospy.loginfo("Mapa visual:")
    for row in map_visual:
        rospy.loginfo(" ".join(row))
    print("\n")

    # Publicar el mapa
    map_pub.publish(map_data)

def warehouse_map_publisher():
    global map_pub

    # Iniciar el nodo
    rospy.init_node('warehouse_map_publisher4', anonymous=True)

    # Publicar el mapa
    map_pub = rospy.Publisher('/warehouse_map', WarehouseMap2, queue_size=10)

    # Suscribirse al estado del robot
    rospy.Subscriber('/robot_status', RobotStatus, robot_status_callback)

    # Publicar el mapa inicial
    publish_map()

    rospy.spin()

if __name__ == '__main__':
    try:
        warehouse_map_publisher()
    except rospy.ROSInterruptException:
        pass