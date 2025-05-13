#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

def warehouse_map_publisher():
    rospy.init_node('warehouse_map_publisher2', anonymous=True)
    pub = rospy.Publisher('/warehouse_map', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(1)

    # Initialize the OccupancyGrid message
    map_msg = OccupancyGrid()

    # Set the header
    map_msg.header = Header()
    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = 'map'

    # Set map resolution and dimensions
    map_msg.info.resolution = 1.0  # meters per cell
    map_msg.info.width = 10  # Grid width (10 units)
    map_msg.info.height = 10  # Grid height (10 units)

    # Initialize the map data (0 for free space, 100 for obstacles)
    map_data = [0] * (map_msg.info.width * map_msg.info.height)

    # Add walls and obstacles to the map (simple representation)
    for i in range(map_msg.info.width):
        map_data[i] = 100  # North wall
        map_data[(map_msg.info.height - 1) * map_msg.info.width + i] = 100  # South wall

    for i in range(map_msg.info.height):
        map_data[i * map_msg.info.width] = 100  # West wall
        map_data[(i + 1) * map_msg.info.width - 1] = 100  # East wall

    # Define charging/unloading stations and shelves (use different values for visualization)
    stations = [
        [0, 2],  # Estante 1 (x, y) - cerca de la parte superior izquierda
        [7, 2],  # Estante 2 (x, y) - cerca de la parte superior derecha
        [0, 7],  # Estante 3 (x, y) - cerca de la parte inferior izquierda
        [7, 7]   # Estante 4 (x, y) - cerca de la parte inferior derecha
    ]

    shelves = [
        [2, 2],  # Estación 1 (x, y) - más cerca de la esquina superior izquierda
        [7, 2],  # Estación 2 (x, y) - más cerca de la parte superior derecha
        [2, 7],  # Estación 3 (x, y) - más centrada
        [7, 7]   # Estación 4 (x, y) - más centrada
    ]

    # Mark stations and shelves with different values
    for station in stations:
        map_data[station[1] * map_msg.info.width + station[0]] = 50  # Estaciones de carga/descarga (valor 50)

    for shelf in shelves:
        map_data[shelf[1] * map_msg.info.width + shelf[0]] = 75  # Estantes (valor 75)

    # Set the map data
    map_msg.data = map_data

    # Publish the map
    while not rospy.is_shutdown():
        pub.publish(map_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        warehouse_map_publisher()
    except rospy.ROSInterruptException:
        pass
