#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import WarehouseMap2, RobotStatus2
from warehouse_robot.srv import CreateMap, CreateMapResponse, CreateMapRequest, GenerateTasks
import random

# Variables globales
map_data = WarehouseMap2()
map_created = False

def robot_status_callback(status_msg):
    global map_data, map_created
    map_data.robot_position = [status_msg.x, status_msg.y]
    if status_msg.paused:
        rospy.loginfo("El robot está pausado.")
    else:
        rospy.loginfo("El robot está reanudado.")
        if map_created:
            publish_map()

def create_map_callback(req):
    global map_data, map_created
    width = req.width
    height = req.height
    num_stations = req.num_stations
    num_shelves = req.num_shelves

    # Limitar el número de estaciones y estanterías
    max_stations = min(num_stations, (width + height) * 2)
    max_shelves = min(num_shelves, (width - 2) * (height - 2))

    # Generar estaciones de carga/descarga en los bordes
    stations = []
    for _ in range(max_stations):
        if random.choice([True, False]):
            stations.extend([random.randint(0, width - 1), random.choice([0, height - 1])])
        else:
            stations.extend([random.choice([0, width - 1]), random.randint(0, height - 1)])

    # Generar estanterías en el interior del almacén
    shelves = []
    for _ in range(max_shelves):
        shelves.extend([random.randint(1, width - 2), random.randint(1, height - 2)])

    # Generar posición inicial del robot en una celda transitable
    while True:
        robot_x = random.randint(1, width - 2)
        robot_y = random.randint(1, height - 2)
        if [robot_x, robot_y] not in shelves:
            break

    # Actualizar el mapa
    map_data.stations = stations
    map_data.shelves = shelves
    map_data.walls = []
    map_data.robot_position = [robot_x, robot_y]

    # Añadir muros en las celdas exteriores, excepto donde haya estaciones de carga/descarga
    for x in range(width):
        if [x, 0] not in stations and [x, height - 1] not in stations:
            map_data.walls.extend([x, 0])  # Pared norte
            map_data.walls.extend([x, height - 1])  # Pared sur
    for y in range(height):
        if [0, y] not in stations and [width - 1, y] not in stations:
            map_data.walls.extend([0, y])  # Pared oeste
            map_data.walls.extend([width - 1, y])  # Pared este

    map_created = True

    # Publicar el nuevo mapa
    publish_map()

    # Llamar al servicio para generar nuevas tareas
    rospy.wait_for_service('generate_tasks')
    try:
        generate_tasks = rospy.ServiceProxy('generate_tasks', GenerateTasks)
        response = generate_tasks(num_tasks=5)  # Puedes ajustar el número de tareas aquí
        rospy.loginfo(response.message)
    except rospy.ServiceException as e:
        rospy.logerr(f"Error al llamar al servicio generate_tasks: {e}")

    return CreateMapResponse(success=True, message="Mapa creado exitosamente")

def publish_map():
    global map_data

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
    robot_x, robot_y = int(map_data.robot_position[0]), int(map_data.robot_position[1])
    map_visual[robot_y][robot_x] = "R"  # Colocamos el robot

    # Mostrar la representación visual del mapa en la consola
    rospy.loginfo("Mapa visual:")
    for row in map_visual:
        rospy.loginfo(" ".join(row))

    # Publicar el mapa
    rospy.loginfo("Publicando el mapa")
    map_pub.publish(map_data)

def warehouse_map_publisher():
    global map_pub

    # Iniciar el nodo
    rospy.init_node('warehouse_map_publisher5', anonymous=True)

    # Leer parámetros
    width = rospy.get_param('~warehouse_width', 20)
    height = rospy.get_param('~warehouse_height', 20)
    num_stations = rospy.get_param('~num_stations', 4)
    num_shelves = rospy.get_param('~num_shelves', 50)

    # Publicar el mapa
    map_pub = rospy.Publisher('/warehouse_map', WarehouseMap2, queue_size=10)

    # Suscribirse al estado del robot
    rospy.Subscriber('/robot_status', RobotStatus2, robot_status_callback)

    # Servicio para crear el mapa
    rospy.Service('create_map', CreateMap, create_map_callback)

    # Crear el mapa por defecto usando los parámetros
    create_map_callback(CreateMapRequest(width=width, height=height, num_stations=num_stations, num_shelves=num_shelves))

    # Esperar a que se reciban las tareas y se publiquen
    rospy.spin()

if __name__ == '__main__':
    try:
        warehouse_map_publisher()
    except rospy.ROSInterruptException:
        pass