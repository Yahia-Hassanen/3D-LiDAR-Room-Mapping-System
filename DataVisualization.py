# 3D Indoor Spatial Mapping — Point Cloud Visualization
# Author: Yahia Hassanen
# Date: March 2026
#
# Description:
# Receives distance measurements from the TM4C1294NCPDT microcontroller
# over UART, converts polar coordinates to Cartesian XYZ, saves the
# point cloud to disk, and renders a 3D wireframe model using Open3D.

# Imports 
import open3d as o3d   # 3D point cloud visualization
import serial          # UART communication with microcontroller
import math            # Trigonometric functions for coordinate conversion
import numpy as np     # Array operations required by Open3D


# Configuration
points = []          # Accumulates [x, y, z] data
samples = 64         # Measurements per full 360-degree sweep
done_scanning = False  
layer_spacing = 100  


# Serial Port Setup 
port = serial.Serial('COM4', 115200, timeout=10)
print("Opening: " + port.name)

# Discard any stale data
port.reset_output_buffer()
port.reset_input_buffer()


# Output File 
output_file = open("./pointcloud.xyz", "w") # XYZ format: one point per line as "x y z" floating point values.


# Synchronization
input("Press Enter to start scanning...")

layer = 0 #Each complete 360-degree sweep is one layer. z increments after each sweep.


# Main  Loop 

while True:
    for i in range(samples):

        # Read distance value from the MCU 
        reading = port.readline().decode().strip()

        # If readline() timed out, keep retrying
        while reading == '':
            reading = port.readline().decode().strip()

        # STOP signal from PJ0 press
        if reading == 'STOP':
            done_scanning = True
            break

        # Convert sample index to angle in degrees
        theta = i * (360.0 / samples)

        # Polar to Cartesian conversion
        x = round(int(reading) * math.cos((math.pi / 180.0) * theta), 1)
        y = round(int(reading) * math.sin((math.pi / 180.0) * theta), 1)

        # Print to terminal for real-time monitoring and debugging
        print(str(x) + " " + str(y) + " " + str(layer) + "\n")

        # Store point in memory and write to file simultaneously
        points.append([x, y, layer * layer_spacing])
        output_file.write("{0:f} {1:f} {2:f}\n".format(x, y, layer * layer_spacing))

    # Exit outer loop if STOP was received during the inner loop
    if done_scanning == True:
        break

    print("Section " + str(layer) + " complete!\n")
    layer = layer + 1   # Advance to next scan position


# Cleanup
print("Closing: " + port.name)
port.close()
output_file.close()


# Load Point Cloud
cloud = o3d.io.read_point_cloud("./pointcloud.xyz", format='xyz')


# Build Wireframe Connections 
edges = []
for i in range(0, len(points), samples):
    for j in range(samples):
        next_j = (j + 1) % samples          # Modulo closes the ring
        edges.append([i + j, i + next_j])

# Inter-layer connections — link each point to the corresponding point one layer up
for i in range(0, len(points) - samples, samples):
    for j in range(samples):
        edges.append([i + j, i + j + samples])


# Render Wireframe 
wireframe = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(np.asarray(cloud.points)),
    lines=o3d.utility.Vector2iVector(edges)
)

o3d.visualization.draw_geometries([wireframe])