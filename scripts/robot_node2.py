#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import Task
from geometry_msgs.msg import Point
import time

def execute_task(task):
    rospy.loginfo(f"Robot {task.robot_id} está empezando la tarea: {task.task_type}")
    rospy.loginfo(f"Moviendo al origen: {task.source}")
    time.sleep(2)  # Simula el tiempo de movimiento
    rospy.loginfo(f"Moviendo al destino: {task.destination}")
    time.sleep(2)  # Simula el tiempo de movimiento
    rospy.loginfo(f"Tarea {task.task_type} completada")

def robot_node():
    rospy.init_node('robot_node', anonymous=True)
    rospy.Subscriber('/task_topic', Task, execute_task)
    rospy.spin()

if __name__ == '__main__':
    try:
        robot_node()
    except rospy.ROSInterruptException:
        pass
