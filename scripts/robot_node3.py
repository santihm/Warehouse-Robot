#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import Task, RobotStatus, WarehouseMap
from geometry_msgs.msg import Point
import time

# Variables globales
current_position = Point(0.0, 0.0, 0.0)  # Posición inicial del robot
current_task = None  # Tarea actual que se está realizando

def warehouse_map_callback(map_data):
    # Este callback recibirá el mapa del almacén
    rospy.loginfo(f"Mapa del almacén recibido con {len(map_data.stations)//2} estaciones y {len(map_data.shelves)//2} estanterías.")
    # Puedes procesar el mapa si es necesario para la navegación más avanzada

def task_callback(task):
    global current_task
    current_task = task
    rospy.loginfo(f"Tarea recibida: {task.task_type}, desde {task.source} hasta {task.destination}")

    # Navegar a la fuente y luego al destino
    navigate_to(task.source, "source")

def navigate_to(destination, point_type):
    global current_position
    rospy.loginfo(f"Robot navegando hacia {point_type}: {destination}")

    # Calcular la ruta paso a paso
    steps = []
    current_x, current_y = current_position.x, current_position.y
    dest_x, dest_y = destination

    while current_x != dest_x or current_y != dest_y:
        if current_x < dest_x:
            current_x += 1
        elif current_x > dest_x:
            current_x -= 1

        if current_y < dest_y:
            current_y += 1
        elif current_y > dest_y:
            current_y -= 1

        steps.append((current_x, current_y))

    # Mover paso a paso
    for step in steps:
        current_position.x, current_position.y = step
        rospy.loginfo(f"Robot en posición: ({current_position.x}, {current_position.y})")
        time.sleep(1)  # Esperar 1 segundo entre cada paso

    # Si hemos llegado a la fuente, navegar al destino
    if point_type == "source":
        rospy.loginfo("Llegamos a la fuente, ahora navegando al destino...")
        navigate_to(current_task.destination, "destination")
    else:
        rospy.loginfo("Llegamos al destino, tarea completada.")
        publish_robot_status("Completed")

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
    rospy.init_node('robot_node3', anonymous=True)

    # Publicar el estado del robot
    robot_status_pub = rospy.Publisher('/robot_status', RobotStatus, queue_size=10)

    # Suscribirse al topic de mapa para obtener la información
    rospy.Subscriber('/warehouse_map', WarehouseMap, warehouse_map_callback)

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