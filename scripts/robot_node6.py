#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import Task, RobotStatus2
from warehouse_robot.srv import PauseResume, PauseResumeResponse
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
import tf
import time
import heapq
import random

# Global variables
current_position = Point(0.0, 0.0, 0.0)  # Initial position of the robot
current_task = None  # Current task being performed
occupancy_grid = None  # Map data
paused = True  # Initial state of the robot is paused

def map_callback(data):
    global occupancy_grid, current_position
    occupancy_grid = data
    rospy.loginfo(f"Map received with size {occupancy_grid.info.width}x{occupancy_grid.info.height}.")

    # Generate initial robot position in a traversable cell
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    while True:
        robot_x = random.randint(1, width - 2)
        robot_y = random.randint(1, height - 2)
        if occupancy_grid.data[robot_y * width + robot_x] == 0:
            current_position.x = robot_x
            current_position.y = robot_y
            break

def task_callback(task):
    global current_task
    current_task = task
    rospy.loginfo(f"Task received: {task.task_type}, from {task.source} to {task.destination}")

    # Navigate to the source and then to the destination
    navigate_to(task.source, "source")

def navigate_to(destination, point_type):
    global current_position, paused, current_task
    if paused:
        rospy.loginfo("The robot is paused.")
        return

    rospy.loginfo(f"Robot navigating to {point_type}: {destination}")

    # Calculate the path using A*
    path = calculate_path((current_position.x, current_position.y), destination)

    # Move step by step
    for step in path:
        if paused:
            rospy.loginfo("The robot is paused.")
            return
        current_position.x, current_position.y = step
        rospy.loginfo(f"Robot at position: ({current_position.x}, {current_position.y})")
        publish_robot_status("InProgress")
        publish_robot_pose()
        publish_transform()
        robot_speed = rospy.get_param('~robot_speed', 1.0)  # Read the parameter dynamically within the loop
        time.sleep(1 / robot_speed)  # Wait according to the robot's speed

    # If we have reached the source, navigate to the destination
    if point_type == "source" and current_task is not None:
        rospy.loginfo("Reached the source, now navigating to the destination...")
        navigate_to(current_task.destination, "destination")
    elif point_type == "destination":
        rospy.loginfo("Reached the destination, task completed.")
        publish_robot_status("Completed")
        current_task = None  # Mark the task as completed

def calculate_path(start, goal):
    # Implement the A* algorithm to find the shortest path
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

def is_transitable(node):
    global occupancy_grid
    x, y = int(node[0]), int(node[1])
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    return 0 <= x < width and 0 <= y < height and occupancy_grid.data[y * width + x] == 0

def publish_robot_status(status):
    global current_position, current_task, paused

    status_msg = RobotStatus2()
    status_msg.robot_id = current_task.robot_id if current_task else "unknown"
    status_msg.x = current_position.x
    status_msg.y = current_position.y
    status_msg.status = status
    status_msg.paused = paused

    robot_status_pub.publish(status_msg)

def publish_robot_pose():
    global current_position

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "map"
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.position.x = current_position.x
    pose_msg.pose.position.y = current_position.y
    pose_msg.pose.position.z = 0
    pose_msg.pose.orientation.w = 1.0

    robot_pose_pub.publish(pose_msg)

def publish_transform():
    global current_position
    br = tf.TransformBroadcaster()
    br.sendTransform((current_position.x, current_position.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "map")

def pause_resume_callback(req):
    global paused
    paused = req.pause
    message = "Robot paused" if paused else "Robot resumed"
    rospy.loginfo(message)
    publish_robot_status("Paused" if paused else "Resumed")
    if not paused and current_task:
        navigate_to(current_task.source, "source")
    return PauseResumeResponse(success=True, message=message)

def robot_node():
    global robot_status_pub, robot_pose_pub

    # Initialize the node
    rospy.init_node('robot_node6', anonymous=True)

    # Read parameters
    robot_speed = rospy.get_param('~robot_speed')

    # Publish the robot's status
    robot_status_pub = rospy.Publisher('/robot_status', RobotStatus2, queue_size=10)

    # Publish the robot's position
    robot_pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)

    # Subscribe to the map topic to get the information
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    # Subscribe to the task topic to receive tasks
    rospy.Subscriber('/task', Task, task_callback)

    # Service to pause/resume the robot
    rospy.Service('pause_resume', PauseResume, pause_resume_callback)

    # Continuously publish the transform
    tf_broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        publish_transform()
        rate.sleep()

    # Wait for tasks to be received and published
    rospy.spin()

if __name__ == '__main__':
    try:
        robot_node()
    except rospy.ROSInterruptException:
        pass