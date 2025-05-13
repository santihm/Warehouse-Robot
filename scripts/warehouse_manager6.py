#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import Task, RobotStatus2
from warehouse_robot.srv import GenerateTasks, GenerateTasksResponse
from nav_msgs.msg import OccupancyGrid
import random

# Global variables
pending_tasks = []  # List of tasks
current_task_index = 0  # Index of the next task to be published
occupancy_grid = None

def map_callback(map_data):
    global occupancy_grid
    occupancy_grid = map_data
    rospy.loginfo(f"Map received with size {occupancy_grid.info.width}x{occupancy_grid.info.height}.")

def generate_tasks_callback(req):
    global pending_tasks, current_task_index, occupancy_grid

    # Forget pending tasks
    pending_tasks = []
    current_task_index = 0

    # Verify that the map is available
    if occupancy_grid is None:
        return GenerateTasksResponse(success=False, message="The map is not available")

    # Generate the tasks
    generate_tasks(req.num_tasks)

    # Publish the first task
    if pending_tasks:
        rospy.loginfo(f"Publishing the first task: {pending_tasks[0].task_type}")
        task_pub.publish(pending_tasks[0])
        current_task_index += 1

    return GenerateTasksResponse(success=True, message="Tasks generated successfully")

def generate_tasks(num_tasks):
    global pending_tasks, occupancy_grid

    width = occupancy_grid.info.width
    height = occupancy_grid.info.height

    # Generate random "Load" or "Unload" tasks
    for i in range(num_tasks):
        task = Task()
        task.robot_id = "R1"
        task.task_type = random.choice(["Load", "Unload"])
        
        # Randomly select a traversable cell for the source and destination
        while True:
            source_x = random.randint(0, width - 1)
            source_y = random.randint(0, height - 1)
            if occupancy_grid.data[source_y * width + source_x] == 0:
                break
        while True:
            dest_x = random.randint(0, width - 1)
            dest_y = random.randint(0, height - 1)
            if occupancy_grid.data[dest_y * width + dest_x] == 0:
                break

        # Convert coordinates to lists of float32 (x, y)
        task.source = [float(source_x), float(source_y)]  # Source
        task.destination = [float(dest_x), float(dest_y)]  # Destination
        
        # Add the task to the list of pending tasks
        pending_tasks.append(task)
    
    rospy.loginfo(f"{len(pending_tasks)} tasks have been generated.")

def robot_status_callback(status_msg):
    global current_task_index, pending_tasks
    
    rospy.loginfo(f"Robot status: {status_msg.robot_id}, Position: ({status_msg.x}, {status_msg.y}), Status: {status_msg.status}")

    # If the robot has completed the task, publish the next task
    if status_msg.status == "Completed" and current_task_index < len(pending_tasks):
        rospy.loginfo(f"Task completed: {pending_tasks[current_task_index - 1].task_type}, Publishing task number {current_task_index + 1}")
        
        # Publish the next task
        task_pub.publish(pending_tasks[current_task_index])
        
        # Increment the task index
        current_task_index += 1
    elif current_task_index == len(pending_tasks):
        rospy.loginfo("All tasks have been completed.")

def warehouse_manager():
    global task_pub

    # Initialize the node
    rospy.init_node('warehouse_manager6', anonymous=True)

    # Publish tasks to the robot
    task_pub = rospy.Publisher('/task', Task, queue_size=10)
    
    # Subscribe to the map topic to get the information
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    # Subscribe to the robot status topic
    rospy.Subscriber('/robot_status', RobotStatus2, robot_status_callback)

    # Service to generate tasks
    rospy.Service('generate_tasks', GenerateTasks, generate_tasks_callback)

    # Wait a moment to make sure everything is ready
    rospy.sleep(1)

    # Generate default tasks
    rospy.wait_for_service('generate_tasks')
    try:
        generate_tasks_service = rospy.ServiceProxy('generate_tasks', GenerateTasks)
        response = generate_tasks_service(num_tasks=5)  # You can adjust the number of tasks here
        rospy.loginfo(response.message)
    except rospy.ServiceException as e:
        rospy.logerr(f"Error calling generate_tasks service: {e}")

    # Wait for tasks to be received and published
    rospy.spin()

if __name__ == '__main__':
    try:
        warehouse_manager()
    except rospy.ROSInterruptException:
        pass