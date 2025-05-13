#!/usr/bin/env python3

import rospy
from warehouse_robot.msg import WarehouseMap2, RobotStatus2
from warehouse_robot.srv import CreateMap, CreateMapResponse, GenerateTasks
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
import random

# Global variables
map_data = WarehouseMap2()
map_created = False

def create_map_callback(req):
    global map_data, map_created
    width = req.width
    height = req.height
    num_stations = req.num_stations
    num_shelves = req.num_shelves

    rospy.loginfo(f"Creating map with width: {width}, height: {height}, stations: {num_stations}, shelves: {num_shelves}")

    # Limit the number of stations and shelves
    max_stations = min(num_stations, (width + height) * 2)
    max_shelves = min(num_shelves, (width - 2) * (height - 2))

    # Generate charging/loading stations on the edges
    stations = []
    for _ in range(max_stations):
        if random.choice([True, False]):
            stations.extend([random.randint(0, width - 1), random.choice([0, height - 1])])
        else:
            stations.extend([random.choice([0, width - 1]), random.randint(0, height - 1)])

    # Generate shelves inside the warehouse
    shelves = []
    for _ in range(max_shelves):
        shelves.extend([random.randint(1, width - 2), random.randint(1, height - 2)])

    # Generate initial robot position in a traversable cell
    while True:
        robot_x = random.randint(1, width - 2)
        robot_y = random.randint(1, height - 2)
        if [robot_x, robot_y] not in shelves and [robot_x, robot_y] not in stations:
            break

    # Update the map
    map_data.stations = stations
    map_data.shelves = shelves
    map_data.walls = []
    map_data.robot_position = [robot_x, robot_y]

    # Add walls on the outer cells, except where there are charging/loading stations
    for x in range(width):
        if [x, 0] not in stations and [x, height - 1] not in stations:
            map_data.walls.extend([x, 0])  # North wall
            map_data.walls.extend([x, height - 1])  # South wall
    for y in range(height):
        if [0, y] not in stations and [width - 1, y] not in stations:
            map_data.walls.extend([0, y])  # West wall
            map_data.walls.extend([width - 1, y])  # East wall

    map_created = True

    # Publish the new map
    publish_map(width, height)

    # Call the service to generate new tasks
    rospy.wait_for_service('generate_tasks')
    try:
        generate_tasks = rospy.ServiceProxy('generate_tasks', GenerateTasks)
        response = generate_tasks(num_tasks=5)  # You can adjust the number of tasks here
        rospy.loginfo(response.message)
    except rospy.ServiceException as e:
        rospy.logerr(f"Error calling generate_tasks service: {e}")

    return CreateMapResponse(success=True, message="Map created successfully")

def publish_map(width, height):
    global map_data

    # Create a visual representation of the map
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.info.resolution = 1.0  # Each cell is 1x1 meter
    occupancy_grid.info.width = width
    occupancy_grid.info.height = height
    occupancy_grid.info.origin = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))

    # Initialize the map with free cells
    occupancy_grid.data = [0] * (occupancy_grid.info.width * occupancy_grid.info.height)

    # Mark the walls
    for i in range(0, len(map_data.walls), 2):
        x, y = int(map_data.walls[i]), int(map_data.walls[i+1])
        occupancy_grid.data[y * occupancy_grid.info.width + x] = 100  # Occupied cell

    # Mark the charging/loading stations
    for i in range(0, len(map_data.stations), 2):
        x, y = int(map_data.stations[i]), int(map_data.stations[i+1])
        occupancy_grid.data[y * occupancy_grid.info.width + x] = 50  # Semi-occupied cell

    # Mark the shelves
    for i in range(0, len(map_data.shelves), 2):
        x, y = int(map_data.shelves[i]), int(map_data.shelves[i+1])
        occupancy_grid.data[y * occupancy_grid.info.width + x] = 75  # Semi-occupied cell

    # Publish the map
    rospy.loginfo("Publishing map")
    map_pub.publish(occupancy_grid)

def warehouse_map_publisher():
    global map_pub

    # Initialize the node
    rospy.init_node('warehouse_map_publisher6', anonymous=True)

    # Read parameters
    width = rospy.get_param('~warehouse_width')
    height = rospy.get_param('~warehouse_height')
    num_stations = rospy.get_param('~num_stations')
    num_shelves = rospy.get_param('~num_shelves')

    # Publish the map
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

    # Service to create the map
    rospy.Service('create_map', CreateMap, create_map_callback)

    # Wait for tasks to be received and published
    rospy.spin()

if __name__ == '__main__':
    try:
        warehouse_map_publisher()
    except rospy.ROSInterruptException:
        pass