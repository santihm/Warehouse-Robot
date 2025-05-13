#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import Task, RobotStatus, WarehouseMap2
from geometry_msgs.msg import Point
import time
import heapq

# Variables globales
current_position = Point(0.0, 0.0, 0.0)  # Posición inicial del robot
current_task = None  # Tarea actual que se está realizando
map_data = None  # Datos del mapa
occupancy_grid = [[0 for _ in range(20)] for _ in range(20)]  # Matriz de ocupación

def warehouse_map_callback(data):
    global current_position, map_data, occupancy_grid
    map_data = data
    current_position.x = data.robot_position[0]
    current_position.y = data.robot_position[1]
    #rospy.loginfo(f"Mapa del almacén recibido con {len(data.stations)//2} estaciones y {len(data.shelves)//2} estanterías.")
    #rospy.loginfo(f"Posición inicial del robot: ({current_position.x}, {current_position.y})")

    # Actualizar la matriz de ocupación
    occupancy_grid = [[0 for _ in range(20)] for _ in range(20)]
    for i in range(0, len(data.walls), 2):
        x, y = int(data.walls[i]), int(data.walls[i+1])
        occupancy_grid[y][x] = 1  # Marcar como no transitable
    for i in range(0, len(data.shelves), 2):
        x, y = int(data.shelves[i]), int(data.shelves[i+1])
        occupancy_grid[y][x] = 1  # Marcar como no transitable

def task_callback(task):
    global current_task
    current_task = task
    rospy.loginfo(f"Tarea recibida: {task.task_type}, desde {task.source} hasta {task.destination}")

    # Navegar a la fuente y luego al destino
    navigate_to(task.source, "source")

def navigate_to(destination, point_type):
    global current_position
    #rospy.loginfo(f"Robot navegando hacia {point_type}: {destination}")

    # Si el destino es una estantería, encontrar una celda adyacente transitable
    if point_type == "destination" and not is_transitable(destination):
        destination = find_adjacent_transitable(destination)

    # Calcular la ruta usando A*
    path = calculate_path((current_position.x, current_position.y), destination)

    # Mover paso a paso
    for step in path:
        current_position.x, current_position.y = step
        #rospy.loginfo(f"Robot en posición: ({current_position.x}, {current_position.y})")
        publish_robot_status("InProgress")
        time.sleep(0.5)  # Esperar 1 segundo entre cada paso

    # Si hemos llegado a la fuente, navegar al destino
    if point_type == "source":
        rospy.loginfo("Llegamos a la fuente, ahora navegando al destino...")
        navigate_to(current_task.destination, "destination")
    else:
        rospy.loginfo("Llegamos al destino, tarea completada.")
        publish_robot_status("Completed")

def find_adjacent_transitable(node):
    x, y = node
    neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
    for n in neighbors:
        if is_transitable(n):
            return n
    return node  # Si no se encuentra ninguna celda adyacente transitable, devolver el nodo original

def is_transitable(node):
    x, y = int(node[0]), int(node[1])
    return 0 <= x < 20 and 0 <= y < 20 and occupancy_grid[y][x] == 0

def calculate_path(start, goal):
    # Implementar el algoritmo A* para encontrar el camino más corto
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(node):
        neighbors = [(node[0] + 1, node[1]), (node[0] - 1, node[1]), (node[0], node[1] + 1), (node[0], node[1] - 1)]
        valid_neighbors = []
        for n in neighbors:
            if is_transitable(n):
                valid_neighbors.append(n)
        return valid_neighbors

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for neighbor in get_neighbors(current):
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []

def publish_robot_status(status):
    global current_position, current_task

    status_msg = RobotStatus()
    status_msg.robot_id = current_task.robot_id
    status_msg.x = current_position.x
    status_msg.y = current_position.y
    status_msg.status = status

    robot_status_pub.publish(status_msg)

def robot_node():
    global robot_status_pub

    # Iniciar el nodo
    rospy.init_node('robot_node4', anonymous=True)

    # Publicar el estado del robot
    robot_status_pub = rospy.Publisher('/robot_status', RobotStatus, queue_size=10)

    # Suscribirse al topic de mapa para obtener la información
    rospy.Subscriber('/warehouse_map', WarehouseMap2, warehouse_map_callback)

    # Suscribirse al topic de tareas para recibir las tareas
    rospy.Subscriber('/task', Task, task_callback)

    # Esperar un momento para asegurarnos de que todo esté listo
    rospy.sleep(1)

    # Esperar a que se reciban las tareas y se publiquen
    rospy.spin()

if __name__ == '__main__':
    try:
        robot_node()
    except rospy.ROSInterruptException:
        pass