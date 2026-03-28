# Python Program to Process XYZ Data
# Author: Yahia Hassanen
# Date: March 2026
#
# Description:
# A Python program to process XYZ data from a 3D LiDAR system, including filtering,
# transformation, and visualization of point cloud data.

#Library imports
import open3d as o3d
import serial
import math
import numpy as np

# List to store 3D scan data points (x, y, z)
scan = []
# Number of angular samples per scan (i.e., how many readings per full 360° rotation)
samples = 64
# Flag to indicate when to stop scanning and begin plotting
begin_plot = False
# Scaling factor for the z coordinate (to separate each scan section in 3D)
offset = 100 #change if needed


# Create and open s
s = serial.Serial('COM5', 115200, timeout=10)
print("Opening: " + s.name)

# Clear and reset any remaining data from the serial port buffers
s.reset_output_buffer()
s.reset_input_buffer()

# Open a file to save the point cloud data in XYZ format
f = open("./pointcloud.xyz", "w")

# Wait for user input before starting the scan
input("Press Enter to start scanning...")

# z is used to represent the current scan section (layer) in the vertical (z) direction
z = 0

while True:
    # Loop over the number of samples for one complete 360° scan
    for i in range(samples):
        # Read one line of data from the serial port, decode to string, and strip whitespace
        data = s.readline().decode().strip()
        # If no data is received, keep reading until data is available
        while data == '':
            data = s.readline().decode().strip()
        # Check for the stop signal from the device
        if data == 'STOP':
            begin_plot = True  # Set flag to begin plotting after finishing this section
            break
        # Calculate the angle (in degrees) corresponding to the current sample index
        angle = i * (360.0 / samples)
        # Convert the distance reading (assumed to be in a suitable integer format) to x and y coordinates
        x = round(int(data) * math.cos((math.pi / 180.0) * angle), 1)
        y = round(int(data) * math.sin((math.pi / 180.0) * angle), 1)
        # Print the computed coordinates to the console (for debugging/monitoring)
        print(str(x) + " " + str(y) + " " + str(z) + "\n")
        # Append the computed 3D point to the scan list; z is scaled by the offset factor
        scan.append([x, y, z * offset])
        # Write the point to the file in a format suitable for point cloud visualization (XYZ format)
        f.write("{0:f} {1:f} {2:f}\n".format(x, y, z * offset))
    # If the stop signal was received, exit the scanning loop
    if begin_plot == True:
        break
    # Inform the user that the current section (layer) is complete
    print("Section " + str(z) + " complete!\n")
    # Increment z to move to the next scan layer
    z = z + 1

# After scanning, close the serial port and file
print("Closing: " + s.name)
s.close()
f.close()

# Read the saved point cloud from the file using Open3D; the file is in XYZ format
pcd = o3d.io.read_point_cloud("./pointcloud.xyz", format='xyz')

# Create a list of vertex indices for the xy slices.
# Each element in xy_slice_vertex is simply a list containing an index corresponding to a point in the scan.
xy_slice_vertex = []
for i in range(0, len(scan)):
    xy_slice_vertex.append([i])
# Replace the whole xy_slice_vertex section with:
lines = []
for i in range(0, len(scan), samples):
    for j in range(samples):
        next_j = (j + 1) % samples
        lines.append([i + j, i + next_j])

for i in range(0, len(scan) - samples, samples):
    for j in range(samples):
        lines.append([i + j, i + j + samples])

# Create an Open3D LineSet object using the scanned points and the computed line segments.
# This will allow visualization of the 3D structure as connected lines.
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),
    lines=o3d.utility.Vector2iVector(lines)
)

# Visualize the line set in an interactive Open3D window.
o3d.visualization.draw_geometries([line_set])


