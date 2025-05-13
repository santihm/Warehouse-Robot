#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import Task
from geometry_msgs.msg import Point

def robot_node():
    rospy.init_node('robot_node', anonymous=True)
    
    # Robot's initial position
    robot_position = Point(0.0, 0.0, 0.0)
    
    # Robot task callback function
    def task_callback(task):
        rospy.loginfo("Robot %s received task: %s", task.robot_id, task.task_type)
        rospy.loginfo("Moving from %s to %s", task.source, task.destination)
        
        # Simulate robot movement: update robot position based on the task
        robot_position.x = task.destination[0]
        robot_position.y = task.destination[1]
        rospy.loginfo("Robot's new position: (%f, %f)", robot_position.x, robot_position.y)

    # Subscribe to the task assignment topic
    rospy.Subscriber('/task_assignment', Task, task_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        robot_node()
    except rospy.ROSInterruptException:
        pass
