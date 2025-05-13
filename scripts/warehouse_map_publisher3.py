#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import WarehouseMap

def warehouse_map_publisher():
    # Iniciar el nodo
    rospy.init_node('warehouse_map_publisher3', anonymous=True)

    # Publicar el mapa
    map_pub = rospy.Publisher('/warehouse_map', WarehouseMap, queue_size=10)

    # Esperar a que ROS esté listo
    rospy.sleep(1)

    # Crear una estructura de mapa de 20x20
    map_data = WarehouseMap()

    # Establecer las estaciones de carga y descarga en el norte y sur
    map_data.stations = [
        9.0, 0.0,  # Estación 1 (norte)
        10.0, 0.0, # Estación 2 (norte)
        9.0, 19.0, # Estación 3 (sur)
        10.0, 19.0 # Estación 4 (sur)
    ]

    # Distribuir las estanterías en diferentes partes del mapa
    map_data.shelves = [
        # Estanterías en la parte superior izquierda
        4.0, 4.0,  # Estante 1
        5.0, 4.0,  # Estante 2

        # Estanterías en la parte superior derecha
        14.0, 4.0, # Estante 3
        15.0, 4.0, # Estante 4

        # Estanterías en la parte inferior izquierda
        4.0, 15.0, # Estante 5
        5.0, 15.0, # Estante 6

        # Estanterías en la parte inferior derecha
        14.0, 15.0, # Estante 7
        15.0, 15.0  # Estante 8
    ]

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

    # Mostrar la representación visual del mapa en la consola
    rospy.loginfo("Mapa visual:")
    for row in map_visual:
        rospy.loginfo(" ".join(row))

    # Publicar el mapa
    map_pub.publish(map_data)

    rospy.spin()

if __name__ == '__main__':
    try:
        warehouse_map_publisher()
    except rospy.ROSInterruptException:
        pass