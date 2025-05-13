#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import Task
from geometry_msgs.msg import Point

def warehouse_manager():
    # Iniciar el nodo
    rospy.init_node('warehouse_manager2', anonymous=True)

    # Publicar el mensaje
    task_pub = rospy.Publisher('/task_topic', Task, queue_size=10)

    # Esperar a que ROS esté listo
    rospy.sleep(1)

    # Definir algunas estaciones de carga/descarga y estanterías
    stations = [
        [0, 0],  # Estación 1 (x, y)
        [9, 0],  # Estación 2 (x, y)
        [0, 3],  # Estación 3 (x, y)
        [9, 3]   # Estación 4 (x, y)
    ]

    shelves = [
        [3, 3],  # Estante 1 (x, y)
        [6, 0],  # Estante 2 (x, y)
        [3, 0],  # Estante 3 (x, y)
        [6, 3]   # Estante 4 (x, y)
    ]

    # Publicar 10 tareas
    for i in range(10):
        task = Task()
        task.robot_id = "R1"
        task.task_type = "Unload" if i % 2 == 0 else "Load"
        
        # Asignar fuente y destino para la tarea
        station = stations[i % len(stations)]
        shelf = shelves[i % len(shelves)]
        
        # Convertir las coordenadas en listas de float32 (x, y)
        task.source = [float(station[0]), float(station[1])]  # Fuente
        task.destination = [float(shelf[0]), float(shelf[1])]  # Destino
        
        # Publicar la tarea
        rospy.loginfo(f"Publicando tarea: robot_id: {task.robot_id}, task_type: {task.task_type}, source: {task.source}, destination: {task.destination}")
        task_pub.publish(task)

        # Esperar un segundo entre publicaciones
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        warehouse_manager()
    except rospy.ROSInterruptException:
        pass
