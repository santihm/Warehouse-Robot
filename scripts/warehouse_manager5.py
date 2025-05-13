#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import Task, WarehouseMap2, RobotStatus2
from warehouse_robot.srv import GenerateTasks, GenerateTasksResponse
import random

# Variables globales
pending_tasks = []  # Lista de tareas
current_task_index = 0  # Índice de la tarea que se publicará a continuación
stations = []
shelves = []

def warehouse_map_callback(map_data):
    global stations, shelves

    # Extraer las estaciones de carga/descarga y las estanterías del mapa
    stations = map_data.stations
    shelves = map_data.shelves

    rospy.loginfo(f"Mapa recibido con {len(stations)//2} estaciones y {len(shelves)//2} estanterías.")

def generate_tasks_callback(req):
    global pending_tasks, current_task_index

    # Olvidar las tareas pendientes
    pending_tasks = []
    current_task_index = 0

    # Verificar que haya suficientes estaciones y estanterías
    if len(stations) < 8 or len(shelves) < 8:
        return GenerateTasksResponse(success=False, message="No hay suficientes estaciones o estanterías para generar tareas")

    # Generar las tareas
    generate_tasks(stations, shelves, req.num_tasks)

    # Publicar la primera tarea
    if pending_tasks:
        rospy.loginfo(f"Publicando la primera tarea: {pending_tasks[0].task_type}")
        task_pub.publish(pending_tasks[0])
        current_task_index += 1

    return GenerateTasksResponse(success=True, message="Tareas generadas exitosamente")

def generate_tasks(stations, shelves, num_tasks):
    global pending_tasks

    # Generar tareas aleatorias de "Load" o "Unload"
    for i in range(num_tasks):
        task = Task()
        task.robot_id = "R1"
        task.task_type = random.choice(["Load", "Unload"])
        
        # Seleccionar aleatoriamente una estación de carga/descarga y una estantería
        station_index = random.randint(0, (len(stations) // 2) - 1) * 2
        shelf_index = random.randint(0, (len(shelves) // 2) - 1) * 2
        
        # Convertir las coordenadas en listas de float32 (x, y)
        task.source = [float(stations[station_index]), float(stations[station_index + 1])]  # Fuente
        task.destination = [float(shelves[shelf_index]), float(shelves[shelf_index + 1])]  # Destino
        
        # Añadir la tarea a la lista de tareas pendientes
        pending_tasks.append(task)
    
    rospy.loginfo(f"Se han generado {len(pending_tasks)} tareas.")

def robot_status_callback(status_msg):
    global current_task_index, pending_tasks
    
    rospy.loginfo(f"Estado del robot: {status_msg.robot_id}, Posición: ({status_msg.x}, {status_msg.y}), Estado: {status_msg.status}")

    # Si el robot ha completado la tarea, publicar la siguiente tarea
    if status_msg.status == "Completed" and current_task_index < len(pending_tasks):
        rospy.loginfo(f"Tarea completada: {pending_tasks[current_task_index - 1].task_type}, Publicando la tarea nº {current_task_index + 1}")
        
        # Publicar la siguiente tarea
        task_pub.publish(pending_tasks[current_task_index])
        
        # Incrementar el índice de tarea
        current_task_index += 1
    elif current_task_index == len(pending_tasks):
        rospy.loginfo("Todas las tareas han sido completadas.")

def warehouse_manager():
    global task_pub

    # Iniciar el nodo
    rospy.init_node('warehouse_manager5', anonymous=True)

    # Publicar las tareas al robot
    task_pub = rospy.Publisher('/task', Task, queue_size=10)
    
    # Suscribirse al topic de mapa para obtener la información
    rospy.Subscriber('/warehouse_map', WarehouseMap2, warehouse_map_callback)

    # Suscribirse al topic de estado del robot
    rospy.Subscriber('/robot_status', RobotStatus2, robot_status_callback)

    # Servicio para generar tareas
    rospy.Service('generate_tasks', GenerateTasks, generate_tasks_callback)

    # Esperar un momento para asegurarnos de que todo esté listo
    rospy.sleep(1)

    # Esperar a que se reciban las tareas y se publiquen
    rospy.spin()

if __name__ == '__main__':
    try:
        warehouse_manager()
    except rospy.ROSInterruptException:
        pass