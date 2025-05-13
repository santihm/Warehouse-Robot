#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import Task

def warehouse_manager():
    rospy.init_node('warehouse_manager', anonymous=True)
    pub = rospy.Publisher('/task_assignment', Task, queue_size=10)
    rate = rospy.Rate(1)

    # Define a sample task: Load from shelf (3, 3) to the north unloading station (1, 0)
    task = Task()
    task.robot_id = "R1"
    task.task_type = "Load"  # Load task
    task.source = (3, 3)  # Shelf location
    task.destination = (1, 0)  # North station

    # Repeat and publish tasks
    while not rospy.is_shutdown():
        pub.publish(task)
        rospy.loginfo("Publishing task: %s", task)
        rate.sleep()

if __name__ == '__main__':
    try:
        warehouse_manager()
    except rospy.ROSInterruptException:
        pass
